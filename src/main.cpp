// Модуль 2.4: брязкіт контактів кнопки, 4 програмні способи + RC-фільтр
// Кнопка: GPIO4 -> кнопка -> GND. LED на GPIO18 перемикається на кожне прийняте натискання.
// RC-варіант: 10 кОм з 3V3 на GPIO4, 100 нФ з GPIO4 на GND, 100 Ом між кнопкою і GND.
//
// Спосіб вибирається клавішею в моніторі порту:
//   1 без захисту · 2 time-based (50 мс, поза ISR) · 3 state-based · 4 polling (5 мс, машина станів)
//   u внутрішня/зовнішня підтяжка · r обнулити лічильники · ? підказка
#include <Arduino.h>

namespace {
    // ── 1. Конфігурація ──────────────────────────────────────────────────────────
    struct Config {
        static constexpr uint8_t BTN_PIN = 4; // кнопка між піном і GND
        static constexpr uint8_t LED_PIN = 18; // індикатор прийнятого натискання

        static constexpr uint32_t DEBOUNCE_MS = 50; // спосіб 2: вікно тиші
        static constexpr uint32_t POLL_MS = 5; // спосіб 4: як часто читати пін
        static constexpr uint8_t STABLE_READS = 4; // спосіб 4: 4 × 5 мс = 20 мс однакового рівня

        static constexpr uint32_t SERIAL_BAUD = 115200;
    };

    enum class Method : uint8_t { Raw = 1, Time = 2, State = 3, Poll = 4 };

    const char *toText(const Method m) {
        switch (m) {
            case Method::Raw: return "1 без захисту";
            case Method::Time: return "2 time-based";
            case Method::State: return "3 state-based";
            case Method::Poll: return "4 polling";
        }

        return "?";
    }

    // ── 2. Те, що пише ISR ─────────────────────────────────
    volatile uint32_t isrEdges = 0; // скільки FALLING побачило залізо загалом
    volatile bool edgePending = false; // «був фронт, loop() ще не забрав»
    volatile uint32_t firstEdgeUs = 0; // мить ПЕРШОГО фронту ще не забраної події

    // ISR: лише лічильник, прапорець і час. Жодного Serial, delay, логіки.
    void IRAM_ATTR onFalling() {
        if (!edgePending) // час фіксуємо лише для першого фронту пачки
            firstEdgeUs = micros();

        edgePending = true;

        ++isrEdges;
    }

    // Забрати подію неподільно (критична секція).
    struct Edge {
        bool pending;
        uint32_t firstUs;
    };

    Edge takeEdge() {
        Edge e{
            .pending = false,
            .firstUs = 0
        };

        noInterrupts();

        if (edgePending) {
            e = {
                .pending = true,
                .firstUs = firstEdgeUs
            };
            edgePending = false;
        }

        interrupts();

        return e;
    }

    uint32_t readIsrEdges() {
        noInterrupts();

        const uint32_t n = isrEdges;

        interrupts();

        return n;
    }

    // ── 3. Стан програми ─────────────────────────────────────────────────────────
    auto method = Method::Raw;
    bool internalPullup = true;
    uint32_t accepted = 0; // скільки натискань прийняв поточний спосіб
    bool ledOn = false;

    bool buttonDown() {
        return digitalRead(Config::BTN_PIN) == LOW;
    } // INPUT_PULLUP: натиснута = 0

    // Спільне «прийняли натискання»: лічильник, LED, рядок у лог.
    void accept(const uint32_t latencyUs) {
        ++accepted;
        ledOn = !ledOn;
        digitalWrite(Config::LED_PIN, ledOn ? HIGH : LOW);

        Serial.printf(
            "[%s] натискань: %lu · фронтів ISR усього: %lu · затримка %lu мкс\n", toText(method),
            static_cast<unsigned long>(accepted), static_cast<unsigned long>(readIsrEdges()),
            static_cast<unsigned long>(latencyUs)
        );
    }

    void resetCounters() {
        noInterrupts();

        isrEdges = 0;
        edgePending = false;

        interrupts();

        accepted = 0;
    }

    void applyInput() {
        pinMode(Config::BTN_PIN, internalPullup ? INPUT_PULLUP : INPUT);
    }

    void setMethod(const Method m) {
        method = m;
        // Спосіб 4 працює без переривань — тоді від'єднуємо ISR повністю.
        if (m == Method::Poll)
            detachInterrupt(digitalPinToInterrupt(Config::BTN_PIN));
        else
            attachInterrupt(digitalPinToInterrupt(Config::BTN_PIN), onFalling, FALLING);

        resetCounters();

        Serial.printf(
            "── спосіб %s · підтяжка %s · лічильники обнулено ──\n", toText(method),
            internalPullup ? "внутрішня" : "зовнішня (INPUT)"
        );
    }

    void printHelp() {
        Serial.println("1 без захисту · 2 time · 3 state · 4 polling · u підтяжка · r обнулити · ? підказка");
    }

    // ── 4. Способи 2, 3, 4 (спосіб 1 — це сам лічильник ISR) ────────────────────
    void runTime(const uint32_t now) {
        static uint32_t lastAccepted = 0;

        const Edge e = takeEdge();
        if (e.pending && now - lastAccepted >= Config::DEBOUNCE_MS) {
            lastAccepted = now;
            accept(micros() - e.firstUs);
        }
    }

    void runState() {
        static bool counted = false; // це натискання вже зараховане?

        const bool down = buttonDown();
        if (!down)
            counted = false; // відпустили → чекаємо нового натискання

        const Edge e = takeEdge();
        if (e.pending && down && !counted) {
            counted = true;
            accept(micros() - e.firstUs);
        }
    }

    // Машина станів: рівень мусить протриматись STABLE_READS читань поспіль.
    enum class PollState : uint8_t { Released, MaybePressed, Pressed, MaybeReleased };

    void runPoll(const uint32_t now) {
        static uint32_t lastRead = 0;
        static auto st = PollState::Released;
        static uint8_t same = 0;
        static uint32_t firstLowUs = 0;

        if (now - lastRead < Config::POLL_MS)
            return;

        lastRead = now;

        const bool down = buttonDown();
        switch (st) {
            case PollState::Released:
                if (down) {
                    st = PollState::MaybePressed;
                    same = 1;
                    firstLowUs = micros();
                }

                break;
            case PollState::MaybePressed:
                if (!down) {
                    st = PollState::Released; // це був відскок
                } else if (++same >= Config::STABLE_READS) {
                    st = PollState::Pressed;
                    accept(micros() - firstLowUs);
                }

                break;
            case PollState::Pressed:
                if (!down) {
                    st = PollState::MaybeReleased;
                    same = 1;
                }

                break;
            case PollState::MaybeReleased:
                if (down) {
                    st = PollState::Pressed; // відскок відпускання
                } else if (++same >= Config::STABLE_READS) {
                    st = PollState::Released;
                }

                break;
        }
    }

    void handleKeys() {
        if (!Serial.available())
            return;

        switch (Serial.read()) {
            case '1':
                setMethod(Method::Raw);

                break;
            case '2':
                setMethod(Method::Time);

                break;
            case '3':
                setMethod(Method::State);

                break;
            case '4':
                setMethod(Method::Poll);

                break;
            case 'u':
                internalPullup = !internalPullup;

                applyInput();
                setMethod(method);

                break;
            case 'r':
                resetCounters();

                Serial.println("── лічильники обнулено ──");

                break;
            case '?':
                printHelp();

                break;
            default:
                break; // Enter, пробіли тощо — ігноруємо
        }
    }
} // namespace

void setup() {
    Serial.begin(Config::SERIAL_BAUD);

    pinMode(Config::LED_PIN, OUTPUT);
    digitalWrite(Config::LED_PIN, LOW);

    applyInput();

    setMethod(Method::Raw); // переривання підключається тут, останнім

    printHelp();
}

void loop() {
    const uint32_t now = millis();

    handleKeys();

    switch (method) {
        case Method::Raw: {
            // Без захисту: кожен фронт = натискання. Друкуємо, коли лічильник змінився.
            static uint32_t shown = 0;

            takeEdge(); // прапорець тут не потрібен — лише скидаємо

            const uint32_t edges = readIsrEdges();

            if (edges != shown) {
                shown = edges;
                accepted = edges;
                Serial.printf(
                    "[%s] натискань (= фронтів): %lu\n", toText(method), static_cast<unsigned long>(edges)
                );
            }

            break;
        }
        case Method::Time:
            runTime(now);

            break;
        case Method::State:
            runState();

            break;
        case Method::Poll:
            runPoll(now);

            break;
    }
}
