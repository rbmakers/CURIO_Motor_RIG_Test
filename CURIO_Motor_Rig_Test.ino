#include <Arduino.h>
#include <Wire.h>
#include <string.h>   // strcmp() — 取代 String 類別，避免長時間運行中的堆積記憶體碎片化
#include <ctype.h>    // toupper() — 指令大小寫正規化
#include "BMI088.h"

/*
 * ============================================================================
 * 物理測試模型說明（重要！這決定了下面所有測試 Pattern 的安全邊界設計）
 * ----------------------------------------------------------------------------
 * 本測試「不是」把四軸測試架完全鎖固、靠剛性結構承受力矩。
 * 實際測試條件是：四顆馬達裝在一台完整的四軸機架上，這台機架本身被安裝在
 * 一個三軸可「自由旋轉」的 Gimbal 測試台上（只限制平移，不限制旋轉）。
 *
 * 核心量測原理：
 *   若四顆「規格相同」的馬達被給予完全相同的 PWM，理論上四個推力應互相
 *   平衡，Gimbal 上的機架不應該有明顯轉動。
 *   --> 任何「非預期的轉動」，代表馬達彼此的實際推力/扭矩特徵不一致
 *       （即使標稱規格相同），這正是本測試要量化的「品質不穩」。
 *   （此處假設馬達接頭與 MOSFET 驅動差異可忽略，量到的差異歸因於馬達本身。）
 *
 * 因此衍生的安全設計準則：
 *   Gimbal 是「自由旋轉」，沒有像鎖固結構那樣的剛性回正力。任何測試
 *   Pattern 都不能讓單顆馬達從 0 直接跳到大 Duty、而其餘三顆完全停轉——
 *   這樣只有一顆馬達在推、零抗衡，會讓機架產生劇烈、大角度甚至失控的
 *   轉動，不只破壞「小信號線性響應」的量測假設，也有撞擊 Gimbal 機構
 *   極限的風險。
 *
 *   故 Pattern 2～5 全部採用「繞 HOVER_BASELINE_DUTY 工作點做小幅度擾動」
 *   的設計：四顆馬達先一起穩定在同一個基準 Duty（模擬接近懸停的工作點），
 *   再對單顆（或對角成對）馬達疊加一個侷限在安全範圍內的位移量。瞬時
 *   不平衡力矩永遠被限制在這個位移量等級，機架只會小幅、可預期、可回復
 *   地傾轉，不會劇烈翻轉。這同時也更貼近實際飛控只在工作點附近做小信號
 *   修正的情境，比「從靜止驟然全幅啟動」更有控制工程上的意義。
 * ============================================================================
 */

// ==========================================
// 🛠️ 硬體接腳與結構定義
// ==========================================
const int M1_PIN = 0;   // 右前 (CW)
const int M2_PIN = 11;  // 右後 (CCW)
const int M3_PIN = 14;  // 左後 (CW)
const int M4_PIN = 28;  // 左前 (CCW)

const int LED_STATUS = 7; // 狀態指示燈

const int I2C0_SDA = 20;
const int I2C0_SCL = 25;
#define BMI088_ACC_ADDR  0x18  
#define BMI088_GYRO_ADDR 0x69  

// ELRS / CRSF 接收機（CURIO UART1，與 CURIO_ELRS_Test.ino 共用同一組接腳/鮑率）
#define ELRS_SERIAL     Serial2
#define ELRS_BAUD       420000        // CRSF 標準鮑率
#define PIN_ELRS_TX     4             // CURIO TX1 = GPIO4
#define PIN_ELRS_RX     5             // CURIO RX1 = GPIO5

// ==========================================
// 📊 實驗測試參數配置
// ==========================================
const int PWM_FREQ = 20000;       // 20kHz 高頻驅動
const int PWM_RANGE = 255;        // 8-bit 解析度

// 加速度計取樣節流：對應 setAccOutputDataRate(ODR_100) = 100Hz（10ms 更新一次）。
// 主迴圈遠快於此速率，若每次都重新讀取，多數時間讀到的只是尚未更新的舊值，
// 白白佔用 I2C 匯流排時間；陀螺儀仍維持每迴圈讀取，不受此節流影響。
const unsigned long ACC_SAMPLE_INTERVAL_US = 10000;

// 共用基準工作點：Pattern 2～5 都繞著這個「貼近懸停」的代表性 Duty 做小信號擾動
// （理由見上方「物理測試模型說明」）。
const int HOVER_BASELINE_DUTY = 76; // ≈ 30%

// Pattern 1：交錯三角波（四顆馬達同步追蹤同一目標值，0% → 50% → 0%）
const int P1_DUTY_MAX = 128;      // 約 50% Duty
const int P1_TOTAL_CYCLES = 300;  // 來回 300 次

// Pattern 2：單馬達小信號階躍響應（繞 HOVER_BASELINE_DUTY 擾動）
const int P2_DUTY_DELTA = 38;          // 階躍偏移量 ≈ 15%，與 P3 同級，風險一致
const int P2_STEP_DURATION_MS = 800;   // 每次脈衝持續時間
const int P2_TOTAL_CYCLES = 50;        // 每顆馬達輪流脈衝 50 次

// Pattern 3：對角虛擬差動階躍（CW 對角 vs CCW 對角，Roll/Pitch 理論上互相抵消）
const int P3_DUTY_DELTA = 38;          // 差動衝擊量 ≈ 15%
const int P3_STEP_DURATION_MS = 600;
const int P3_TOTAL_CYCLES = 50;        // 兩組對角線交替衝擊 50 次

// Pattern 4：低幅度漸增死區掃描（單馬達，繞基準以 1 LSB 步階緩慢爬升找響應門檻）
const int P4_SWEEP_MAX_DELTA = 40;             // 基準之上最多 +40 LSB ≈ 15.7%，
                                                // 留有安全餘裕，同時足以涵蓋常見死區範圍
const unsigned long P4_STEP_INTERVAL_MS = 200; // 每 1 LSB 停留時間，利於準靜態判讀

// Pattern 5：頻率掃描正弦波（單馬達，線性 Chirp，0.5Hz → 15Hz）
const int P5_CHIRP_AMPLITUDE = 30;        // 振幅 ≈ 11.8%，較 P2/P3 保守，降低長時間正弦下的疲勞風險
const float P5_FREQ_START_HZ = 0.5f;
const float P5_FREQ_END_HZ   = 15.0f;
const float P5_SWEEP_DURATION_S = 20.0f;  // 每顆馬達掃描時長

// ==========================================
// 🎮 ELRS / CRSF 遙控解析（移植自 CURIO_ELRS_Test.ino）
// ==========================================
/*
 * 操作流程（對應 TX16S 實機操作）：
 *   1. 通電 → CURIO 與 TX16S 完成 ELRS BIND → 開始收到 CRSF 訊框，此時 CH5 應為低位（DISARM）。
 *   2. 操作者把 TX16S 的 CH5 開關撥到高位 → 進入 ARMING，開始倒數 ARM_HOLD_MS。
 *   3. 倒數期間若 CH5 提前放手（回到低位）→ 取消，回到 DISARM。
 *   4. 倒數滿 ARM_HOLD_MS 仍維持高位 → 正式 ARMED，自動觸發 Motor Rig Test 主程式
 *     （等同於原本手動輸入 'START'）。
 *   5. 測試運行中，只要 CH5 回到低位、或 ELRS 訊號中斷超過 CRSF_LINK_TIMEOUT_MS，
 *      立即視為安全中斷：強制停機、回到 DISARM。
 *   6. 測試「自然跑完」(五階段全部結束) 後，同樣要求操作者把 CH5 重新扳低再扳高一次
 *      才能再次觸發——避免開關仍停在高位時被誤判成新一輪 ARM 而自動重新啟動整套 20 分鐘測試。
 *
 * 這個 ARM 狀態機是獨立於 current_pattern 之上的一層安全閘門：current_pattern 只決定
 * 「測試跑到哪個階段」，arm_state 才決定「現在到底有沒有被允許讓馬達轉」。
 */
#define CRSF_SYNC_BYTE     0xC8
#define CRSF_TYPE_RC_CHAN  0x16
#define CRSF_MAX_FRAME_LEN 64

static uint8_t  crsfBuf[CRSF_MAX_FRAME_LEN];
static uint8_t  crsfBufIdx  = 0;
static uint8_t  crsfExpLen  = 0;
static bool     crsfInFrame = false;

static unsigned long channels[16];
static uint32_t frameCount    = 0;
static uint32_t crcErrorCount = 0;
static uint32_t lastFrameMs   = 0;

const int CH5_INDEX = 4; // channels[] 為 0-indexed，CH5（Arm 開關）對應 index 4

// ARM 安全參數（皆可依實際 TX16S 校正值調整）
const unsigned long CRSF_LINK_TIMEOUT_MS    = 500;  // 超過此時間沒收到新 CRSF 訊框，視為斷線
const unsigned long CH5_ARM_THRESHOLD_US    = 1700; // 高於此值視為「開關切高」(意圖 ARM)
const unsigned long CH5_DISARM_THRESHOLD_US = 1300; // 低於此值視為「開關切低」(DISARM)
                                                     // 兩者間留 400us 的死區，避免開關邊界訊號抖動誤判
const unsigned long ARM_HOLD_MS = 1500;             // 切高後需維持的確認時間（落在 1~2 秒區間中點）

enum ArmState {
    ARM_DISARMED,  // CH5 低或無訊號，馬達禁止轉動
    ARM_ARMING,    // CH5 已切高，正在等待 ARM_HOLD_MS 確認時間
    ARM_ARMED      // 確認時間已滿，主測試程式可以運行
};
ArmState arm_state = ARM_DISARMED;
unsigned long arming_start_ms = 0;

// true 時即使 CH5 目前是高位也不允許進入 ARMING，必須先看到一次低位才解除。
// 用於避免「測試自然跑完」或「Serial STOP」之後，開關仍停在高位卻被誤判成新一輪 ARM。
bool ch5_require_cycle = false;

// ==========================================
// 📐 姿態感測器與 Mahony 變數
// ==========================================
BMI088 bmi088(Wire, BMI088_ACC_ADDR, BMI088_GYRO_ADDR);
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

#define kp_mahony 2.0f
#define ki_mahony 0.005f
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float eIntX = 0.0f, eIntY = 0.0f, eIntZ = 0.0f;
unsigned long lastUpdate = 0;
unsigned long lastAccelUpdate = 0; // 加速度計取樣節流計時
float roll = 0.0f, pitch = 0.0f;

// ==========================================
// ⏱️ 自動化實驗排程狀態機
// ==========================================
enum TestPattern {
    PAT_IDLE,
    PAT_TRIANGLE,           // 階段一：交錯三角波
    PAT_DEADZONE_SWEEP,     // 階段二：低幅度漸增死區掃描
    PAT_STEP_RESPONSE,      // 階段三：單馬達小信號階躍響應
    PAT_CHIRP_SWEEP,        // 階段四：頻率掃描正弦波 (Chirp)
    PAT_DIFFERENTIAL_STEP,  // 階段五：對角虛擬差動階躍
    PAT_COOLDOWN,           // 通用過渡冷卻（由 next_pattern_after_cooldown 決定下一站）
    PAT_DONE                // 測試結束
};
TestPattern current_pattern = PAT_IDLE;
TestPattern next_pattern_after_cooldown = PAT_IDLE; // 冷卻結束後該進入哪個階段

// 內部計數與時間追蹤變數
int global_cycle = 0;
int sub_step = 0;          // 用於多階段細部計數（例如目前測到第幾顆馬達）
unsigned long pattern_timer = 0;
int global_target_duty = 0;
int duty_direction = 1;
int loop_counter = 0;
unsigned long last_loop_time = 0;

// 指令解析緩衝區：以固定長度 char buffer 取代 Arduino String 與
// Serial.readStringUntil()。原因：readStringUntil 在收到第一個位元組但換行符
// 尚未送達時，會依預設逾時（1000ms）整個阻塞 loop()——也就是阻塞 IMU 取樣、
// Mahony 解算與馬達 PWM 更新長達 1 秒。對一個要求 2ms 級時間精度的系統辨識
// 實驗來說，這個風險必須消除；改為逐位元組非阻塞收集也順便避免了長時間測試
// (~20 分鐘) 中 String 動態配置/釋放造成的堆積記憶體碎片化。
char cmdBuf[16];
uint8_t cmdLen = 0;

// ==========================================
// 🛠️ 核心工具函數
// ==========================================

// 統一的馬達輸出函式：所有 Pattern 共用同一個出口，並用 constrain() 做保險絲——
// 即使日後調整測試參數造成計算值溢出，也不會送出超出 PWM 解析度範圍的 Duty。
void setMotors(int m1, int m2, int m3, int m4) {
    analogWrite(M1_PIN, constrain(m1, 0, PWM_RANGE));
    analogWrite(M2_PIN, constrain(m2, 0, PWM_RANGE));
    analogWrite(M3_PIN, constrain(m3, 0, PWM_RANGE));
    analogWrite(M4_PIN, constrain(m4, 0, PWM_RANGE));
}

void stopAllMotors() {
    setMotors(0, 0, 0, 0);
}

// ---- CRSF 協議解析（與 CURIO_ELRS_Test.ino 完全相同的實作，直接移植）----

static uint8_t crsfCrc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
        }
    }
    return crc;
}

static inline unsigned long crsfToUs(uint16_t raw) {
    return (unsigned long)constrain((long)raw * 1000L / 1639L + 895L, 1000L, 2000L);
}

static void crsfDecodeChannels(const uint8_t *p) {
    uint16_t ch[16];
    ch[0]  = ((uint16_t)(p[0])       | (uint16_t)(p[1])  << 8) & 0x07FF;
    ch[1]  = ((uint16_t)(p[1])  >> 3 | (uint16_t)(p[2])  << 5) & 0x07FF;
    ch[2]  = ((uint16_t)(p[2])  >> 6 | (uint16_t)(p[3])  << 2 | (uint16_t)(p[4]) << 10) & 0x07FF;
    ch[3]  = ((uint16_t)(p[4])  >> 1 | (uint16_t)(p[5])  << 7) & 0x07FF;
    ch[4]  = ((uint16_t)(p[5])  >> 4 | (uint16_t)(p[6])  << 4) & 0x07FF;
    ch[5]  = ((uint16_t)(p[6])  >> 7 | (uint16_t)(p[7])  << 1 | (uint16_t)(p[8]) << 9) & 0x07FF;
    ch[6]  = ((uint16_t)(p[8])  >> 2 | (uint16_t)(p[9])  << 6) & 0x07FF;
    ch[7]  = ((uint16_t)(p[9])  >> 5 | (uint16_t)(p[10]) << 3) & 0x07FF;
    ch[8]  = ((uint16_t)(p[11])      | (uint16_t)(p[12]) << 8) & 0x07FF;
    ch[9]  = ((uint16_t)(p[12]) >> 3 | (uint16_t)(p[13]) << 5) & 0x07FF;
    ch[10] = ((uint16_t)(p[13]) >> 6 | (uint16_t)(p[14]) << 2 | (uint16_t)(p[15]) << 10) & 0x07FF;
    ch[11] = ((uint16_t)(p[15]) >> 1 | (uint16_t)(p[16]) << 7) & 0x07FF;
    ch[12] = ((uint16_t)(p[16]) >> 4 | (uint16_t)(p[17]) << 4) & 0x07FF;
    ch[13] = ((uint16_t)(p[17]) >> 7 | (uint16_t)(p[18]) << 1 | (uint16_t)(p[19]) << 9) & 0x07FF;
    ch[14] = ((uint16_t)(p[19]) >> 2 | (uint16_t)(p[20]) << 6) & 0x07FF;
    ch[15] = ((uint16_t)(p[20]) >> 5 | (uint16_t)(p[21]) << 3) & 0x07FF;

    for (int i = 0; i < 16; i++) {
        channels[i] = crsfToUs(ch[i]);
    }
    frameCount++;
    lastFrameMs = millis();
}

// 持續性地把 UART RX 緩衝區排空、組訊框、驗證 CRC、解碼——必須每次 loop() 都呼叫，
// 不可被狀態機的 2ms 節流卡住，否則硬體 UART 緩衝區會在收到下一個訊框前被覆寫/溢位。
void parseCRSF() {
    while (ELRS_SERIAL.available()) {
        uint8_t byte = ELRS_SERIAL.read();
        if (!crsfInFrame) {
            if (byte == CRSF_SYNC_BYTE) {
                crsfInFrame = true;
                crsfBufIdx  = 0;
                crsfBuf[crsfBufIdx++] = byte;
            }
        } else {
            crsfBuf[crsfBufIdx++] = byte;
            if (crsfBufIdx == 2) {
                crsfExpLen = byte + 2;
                if (crsfExpLen > CRSF_MAX_FRAME_LEN) {
                    crsfInFrame = false;
                }
            }
            if (crsfInFrame && crsfExpLen > 0 && crsfBufIdx >= crsfExpLen) {
                uint8_t rxCrc  = crsfBuf[crsfExpLen - 1];
                uint8_t calCrc = crsfCrc8(&crsfBuf[2], crsfExpLen - 3);
                if (rxCrc == calCrc) {
                    if (crsfBuf[2] == CRSF_TYPE_RC_CHAN) {
                        crsfDecodeChannels(&crsfBuf[3]);
                    }
                } else {
                    crcErrorCount++;
                }
                crsfInFrame = false;
            }
        }
    }
}

void calibrateSensors() {
    Serial.println("保持測試架絕對靜止，正在校正 IMU 基準值...");
    long samples = 500;
    for(int i=0; i<samples; i++) {
        bmi088.getAcceleration(&ax, &ay, &az);
        bmi088.getGyroscope(&gx, &gy, &gz);
        ax_offset += ax; ay_offset += ay; az_offset += (az - 9.80665f);
        gx_offset += gx; gy_offset += gy; gz_offset += gz;
        delay(3);
    }
    ax_offset /= samples; ay_offset /= samples; az_offset /= samples;
    gx_offset /= samples; gy_offset /= samples; gz_offset /= samples;
    Serial.println("✅ 基準校正完成！請將 TX16S CH5 開關切高並維持 1.5 秒以 ARM 並啟動五階段測試，");
    Serial.println("   或在未連接遙控器時直接輸入 'START'（僅供無 ELRS 連線的單機測試使用）。");
}

void resetMahonyFilter() {
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    eIntX = 0.0f; eIntY = 0.0f; eIntZ = 0.0f;
    roll = 0.0f; pitch = 0.0f;
}

void updateMahony(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;

    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        if(ki_mahony > 0.0f) {
            eIntX += halfex * ki_mahony * dt;
            eIntY += halfey * ki_mahony * dt;
            eIntZ += halfez * ki_mahony * dt;
            gx += eIntX; gy += eIntY; gz += eIntZ;
        }
        gx += halfex * kp_mahony;
        gy += halfey * kp_mahony;
        gz += halfez * kp_mahony;
    }

    gx *= (0.5f * dt); gy *= (0.5f * dt); gz *= (0.5f * dt);
    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);
    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
    
    roll  = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * 57.29578f;
    pitch = asinf(-2.0f * (q1*q3 - q0*q2)) * 57.29578f;
}

// 實時數據 CSV 高速打出
//
// 優化說明：原始版本每行呼叫 11 次 Serial.print()，每次呼叫各自有函式呼叫開銷，
// 浮點數轉字串 (Serial.print(float,N)) 又是相對昂貴的運算。在 500Hz（每 2ms 一行）
// 的設計節拍下，零散呼叫的總耗時會直接侵蝕原本要留給狀態機與 IMU 的時間預算，
// 造成實際取樣間隔比理論值更長、更不規則（jitter），破壞系統辨識所需的時間基準。
// 改為 snprintf() 一次性格式化成單一緩衝區，再用一次 Serial.write() 送出，
// 把「組字串」與「I/O」徹底分離，且只有一次函式呼叫開銷。
//
// freqHz：新增的第 12 欄，僅供 Pattern 5（Chirp）填入當前瞬時頻率，其他
// Pattern 呼叫時一律明確傳入 0.0f。
//
// 注意：此處刻意「不」使用預設參數 (freqHz = 0.0f)。Arduino IDE 會用 ctags
// 自動掃描 .ino 並在檔案最前面插入函式原型宣告；若函式定義本身帶有預設值，
// 自動產生的原型也會原樣複製同一個預設值，導致「同一參數被宣告了兩次預設值」
// 而編譯失敗（這正是本專案先前在自訂 struct/enum 上踩過的同一類工具鏈陷阱，
// 這裡改用「呼叫端一律明確傳值」從根源避免）。
void logTestData(const char* label, int m1, int m2, int m3, int m4, float freqHz) {
    char line[160];
    int len = snprintf(line, sizeof(line),
        "%s,%d,%d,%d,%d,%d,%.3f,%.3f,%.2f,%.2f,%.2f,%.3f\n",
        label, global_cycle, m1, m2, m3, m4,
        roll, pitch,
        gx * 57.29578f, gy * 57.29578f, gz * 57.29578f,
        freqHz);

    if (len > 0 && len < (int)sizeof(line)) {
        Serial.write((const uint8_t*)line, len);
    }
}

// 啟動五階段測試的共用進入點：無論是 Serial 'START' 指令或 ELRS CH5 ARM
// 流程觸發，最終都呼叫這一個函式，確保兩條觸發路徑的初始化邏輯完全一致。
void beginMotorRigTest() {
    current_pattern = PAT_TRIANGLE;
    global_cycle = 0;
    sub_step = 0;
    global_target_duty = 0;
    duty_direction = 1;
    loop_counter = 0;
    next_pattern_after_cooldown = PAT_IDLE;
    digitalWrite(LED_STATUS, HIGH);
    // 輸出標準 CSV 表頭（FreqHz 為第 12 欄，僅 Chirp 階段為非零值）
    Serial.println("Pattern,Cycle,M1,M2,M3,M4,Roll,Pitch,GyroX,GyroY,GyroZ,FreqHz");
}

// 將 START/STOP 指令處理邏輯獨立出來，方便日後擴充指令
void handleCommand(char* cmd) {
    for (uint8_t i = 0; cmd[i] != '\0'; i++) cmd[i] = toupper(cmd[i]);

    if (strcmp(cmd, "START") == 0 && current_pattern == PAT_IDLE) {
        bool linked = (millis() - lastFrameMs < CRSF_LINK_TIMEOUT_MS);
        if (linked) {
            // 已偵測到有效的 ELRS 連線時，ARM 權責交給 TX16S CH5 開關，
            // 避免「鍵盤」與「遙控器」兩個控制源同時可以讓馬達轉動。
            Serial.println("⚠️ 已偵測到 ELRS 連線，請使用 TX16S CH5 開關進行 ARM；Serial START 已停用。");
        } else {
            beginMotorRigTest();
        }
    } else if (strcmp(cmd, "STOP") == 0) {
        current_pattern = PAT_IDLE;
        arm_state = ARM_DISARMED;   // 同步重置 ARM 狀態機，避免 CH5 仍停在高位時被誤判成新一輪 ARM
        ch5_require_cycle = true;  // 要求操作者重新扳動開關才能再次觸發
        digitalWrite(LED_STATUS, LOW);
        stopAllMotors();
        Serial.println("⛔ 測試強制終止，馬達安全關閉。");
    }
}

// 非阻塞式逐位元組收集指令，取代 Serial.readStringUntil()
void pollSerialCommand() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmdLen > 0) {
                cmdBuf[cmdLen] = '\0';
                handleCommand(cmdBuf);
                cmdLen = 0;
            }
        } else if (cmdLen < sizeof(cmdBuf) - 1) {
            cmdBuf[cmdLen++] = c;
        }
    }
}

// ARM 安全狀態機：每次 loop() 都呼叫一次，獨立於 2ms 狀態機節拍之外，
// 確保 DISARM／斷線中斷的反應不會被狀態機節拍延遲。
void updateArmState() {
    bool linked   = (millis() - lastFrameMs < CRSF_LINK_TIMEOUT_MS);
    bool ch5_high = linked && (channels[CH5_INDEX] > CH5_ARM_THRESHOLD_US);
    bool ch5_low  = (!linked) || (channels[CH5_INDEX] < CH5_DISARM_THRESHOLD_US);

    if (ch5_low) {
        ch5_require_cycle = false; // 偵測到一次低位，解除「需重新扳動」的要求
    }

    switch (arm_state) {
        case ARM_DISARMED:
            if (ch5_high && !ch5_require_cycle) {
                arm_state = ARM_ARMING;
                arming_start_ms = millis();
                Serial.println("🔶 [ARM] CH5 切高，進入 ARMING 確認倒數...");
            }
            break;

        case ARM_ARMING:
            if (ch5_low) {
                arm_state = ARM_DISARMED;
                Serial.println("⚪ [ARM] CH5 提前放手，取消 ARMING。");
            } else if (millis() - arming_start_ms >= ARM_HOLD_MS) {
                arm_state = ARM_ARMED;
                Serial.println("🟢 [ARM] 確認時間已滿，ARMED！開始執行五階段測試。");
                beginMotorRigTest();
            }
            break;

        case ARM_ARMED:
            if (ch5_low) {
                // 飛控層級的硬中斷：不論測試跑到哪個階段，立即停止
                arm_state = ARM_DISARMED;
                current_pattern = PAT_IDLE;
                stopAllMotors();
                digitalWrite(LED_STATUS, LOW);
                Serial.println("⛔ [ARM] CH5 回到 DISARM（或 ELRS 訊號中斷），測試立即中止。");
            } else if (current_pattern == PAT_IDLE) {
                // 五階段測試已自然跑完（PAT_DONE 已將 current_pattern 設回 PAT_IDLE）。
                // 要求重新扳動開關才能再次觸發，避免開關仍停在高位時誤判成新一輪 ARM。
                arm_state = ARM_DISARMED;
                ch5_require_cycle = true;
                Serial.println("ℹ️ [ARM] 測試已完成，請將 CH5 切回低再切高以重新啟動。");
            }
            break;
    }

    // 待機階段（尚未開始測試）才印狀態，避免干擾測試運行時的 CSV 資料流
    static unsigned long lastStatusPrint = 0;
    if (current_pattern == PAT_IDLE && millis() - lastStatusPrint >= 500) {
        lastStatusPrint = millis();
        Serial.print("[ELRS] Link=");
        Serial.print(linked ? "OK " : "-- ");
        Serial.print(" CH5=");
        Serial.print(channels[CH5_INDEX]);
        Serial.print("us  ArmState=");
        Serial.println(arm_state == ARM_DISARMED ? "DISARMED" :
                        arm_state == ARM_ARMING   ? "ARMING"   : "ARMED");
    }
}

// ==========================================
// 🕒 初始化與主程式
// ==========================================
void setup() {
    Serial.begin(115200);
    pinMode(LED_STATUS, OUTPUT);
    digitalWrite(LED_STATUS, LOW);

    ELRS_SERIAL.setTX(PIN_ELRS_TX);
    ELRS_SERIAL.setRX(PIN_ELRS_RX);
    ELRS_SERIAL.begin(ELRS_BAUD);

    Wire.setSDA(I2C0_SDA); Wire.setSCL(I2C0_SCL);
    Wire.begin();
    Wire.setClock(400000);

    while(!bmi088.isConnection()) { delay(100); }
    bmi088.initialize();
    calibrateSensors();

    analogWriteFreq(PWM_FREQ);
    analogWriteRange(PWM_RANGE);
    stopAllMotors();

    lastUpdate = micros();
    lastAccelUpdate = micros();
    last_loop_time = millis();
}

void loop() {
    // 1. 命令監聽（非阻塞）：序列埠手動指令 + ELRS/CRSF 遙控訊框
    pollSerialCommand();
    parseCRSF();        // 必須每次 loop() 都排空 UART RX，避免硬體緩衝區溢位
    updateArmState();    // CH5 ARM/DISARM 安全狀態機，獨立於 2ms 測試節拍之外即時反應

    // 2. 高頻 IMU 更新與姿態解算
    // 陀螺儀每迴圈都讀（符合 ODR_2000_BW_532 的高更新率，維持積分精度）；
    // 加速度計依其實際 ODR_100（10ms）節流讀取，避免重複讀到尚未更新的舊資料，
    // 省下的 I2C 匯流排時間留給後面的狀態機與 Serial 輸出。
    unsigned long now_accel_u = micros();
    if (now_accel_u - lastAccelUpdate >= ACC_SAMPLE_INTERVAL_US) {
        lastAccelUpdate = now_accel_u;
        bmi088.getAcceleration(&ax, &ay, &az);
        ax -= ax_offset; ay -= ay_offset; az -= az_offset;
    }
    bmi088.getGyroscope(&gx, &gy, &gz);
    gx -= gx_offset; gy -= gy_offset; gz -= gz_offset;

    unsigned long now_u = micros();
    float dt = (now_u - lastUpdate) / 1000000.0f;
    lastUpdate = now_u;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.002f;
    updateMahony(ax, ay, az, gx, gy, gz, dt);

    // 3. 核心 500Hz 自動排程測試狀態機
    unsigned long now_ms = millis();
    if (now_ms - last_loop_time >= 2) { // 嚴格 2ms 步進
        last_loop_time = now_ms;

        int out_m1 = 0, out_m2 = 0, out_m3 = 0, out_m4 = 0;

        switch (current_pattern) {

            // ------------------------------------------------------------
            // 📊 階段一：三角波交叉遞增測試 (0% -> 50% -> 0%)
            // 四顆馬達同步追蹤同一目標值（僅有 round-robin 寫入造成的
            // 數毫秒相位差），全程量測「同 PWM 是否同推力」這個核心假設
            // 在整個油門範圍內是否成立。
            // ------------------------------------------------------------
            case PAT_TRIANGLE: {
                int motor_idx = loop_counter % 4;
                if (motor_idx == 0) {
                    global_target_duty += duty_direction;
                    if (global_target_duty >= P1_DUTY_MAX) {
                        global_target_duty = P1_DUTY_MAX;
                        duty_direction = -1;
                    } else if (global_target_duty <= 0) {
                        global_target_duty = 0;
                        duty_direction = 1;
                        global_cycle++;
                        if (global_cycle >= P1_TOTAL_CYCLES) {
                            stopAllMotors();
                            current_pattern = PAT_COOLDOWN;
                            next_pattern_after_cooldown = PAT_DEADZONE_SWEEP;
                            pattern_timer = millis();
                            sub_step = 0;
                            break;
                        }
                    }
                }

                // 四個主迴圈輪流完成一次步進更新
                static int p1_d[4] = {0,0,0,0};
                p1_d[motor_idx] = global_target_duty;
                out_m1 = p1_d[0]; out_m2 = p1_d[1]; out_m3 = p1_d[2]; out_m4 = p1_d[3];

                setMotors(out_m1, out_m2, out_m3, out_m4);

                logTestData("Triangle", out_m1, out_m2, out_m3, out_m4, 0.0f);
                loop_counter++;
                break;
            }

            // ------------------------------------------------------------
            // 📊 階段二：低幅度漸增死區掃描 (Dead-zone Sweep)
            // 單顆馬達為一組（sub_step 0~3 對應 M1~M4），其餘三顆與待測馬達
            // 一起先停在 HOVER_BASELINE_DUTY，再對待測馬達每 200ms 緩慢疊加
            // 1 LSB，直到 +P4_SWEEP_MAX_DELTA 為止。量到的是「相對於共同
            // 基準的增量響應門檻」——若某顆馬達的換向器/軸承靜摩擦較大，
            // 需要更大的位移量才會讓機架開始出現可辨識的傾轉。
            // ------------------------------------------------------------
            case PAT_DEADZONE_SWEEP: {
                unsigned long elapsed = millis() - pattern_timer;
                int delta = elapsed / P4_STEP_INTERVAL_MS; // 每 200ms 增加 1 LSB

                char dz_label[16];
                sprintf(dz_label, "Deadzone_M%d", sub_step + 1);

                out_m1 = out_m2 = out_m3 = out_m4 = HOVER_BASELINE_DUTY;

                if (delta > P4_SWEEP_MAX_DELTA) {
                    // 本顆馬達掃描完畢：先記錄一筆回到基準的列，再切換下一顆
                    setMotors(out_m1, out_m2, out_m3, out_m4);
                    logTestData(dz_label, out_m1, out_m2, out_m3, out_m4, 0.0f);

                    sub_step++;
                    pattern_timer = millis();
                    if (sub_step >= 4) { // 四顆馬達全部掃完
                        stopAllMotors();
                        current_pattern = PAT_COOLDOWN;
                        next_pattern_after_cooldown = PAT_STEP_RESPONSE;
                        sub_step = 0;
                    }
                    break;
                }

                int swept_duty = HOVER_BASELINE_DUTY + delta;
                if (sub_step == 0) out_m1 = swept_duty;
                else if (sub_step == 1) out_m2 = swept_duty;
                else if (sub_step == 2) out_m3 = swept_duty;
                else if (sub_step == 3) out_m4 = swept_duty;

                setMotors(out_m1, out_m2, out_m3, out_m4);
                logTestData(dz_label, out_m1, out_m2, out_m3, out_m4, 0.0f);
                break;
            }

            // ------------------------------------------------------------
            // ⏳ 通用過渡冷卻階段 (10 秒強制歸零靜止 + 後段軟體重置)
            // 結束時依 next_pattern_after_cooldown 決定進入哪個下一階段，
            // 新增測試 Pattern 時不需要再額外新增 COOLDOWN_N 列舉值。
            // ------------------------------------------------------------
            case PAT_COOLDOWN: {
                stopAllMotors();
                unsigned long elapsed = millis() - pattern_timer;

                // 前 8 秒純靜止等待，後 2 秒持續重置 Mahony 濾波器確保絕對歸零
                if (elapsed > 8000) {
                    resetMahonyFilter();
                }

                // 實時輸出冷卻狀態，讓資料分析時有明顯的隔離帶
                logTestData("Cooldown", 0, 0, 0, 0, 0.0f);

                if (elapsed >= 10000) { // 10秒時間到，推進至下一階段
                    global_cycle = 0;
                    sub_step = 0;
                    pattern_timer = millis();
                    current_pattern = next_pattern_after_cooldown;
                }
                break;
            }

            // ------------------------------------------------------------
            // 📊 階段三：單馬達小信號階躍響應 (Step Response)
            // 四顆馬達先一起停在 HOVER_BASELINE_DUTY，待測馬達（sub_step
            // 0~3 對應 M1~M4）疊加 +P2_DUTY_DELTA 維持半個週期再回到基準，
            // 其餘三顆全程維持在基準——瞬時不平衡力矩被限制在 P2_DUTY_DELTA
            // 範圍內，機架只會小幅可控地傾轉，不會因為「單顆全幅、其餘
            // 停轉」而劇烈翻轉。
            // ------------------------------------------------------------
            case PAT_STEP_RESPONSE: {
                unsigned long elapsed_step = millis() - pattern_timer;

                out_m1 = out_m2 = out_m3 = out_m4 = HOVER_BASELINE_DUTY;

                if (elapsed_step < P2_STEP_DURATION_MS) {
                    int stepped_duty = HOVER_BASELINE_DUTY + P2_DUTY_DELTA;
                    if (sub_step == 0) out_m1 = stepped_duty;
                    else if (sub_step == 1) out_m2 = stepped_duty;
                    else if (sub_step == 2) out_m3 = stepped_duty;
                    else if (sub_step == 3) out_m4 = stepped_duty;
                } else if (elapsed_step >= P2_STEP_DURATION_MS * 2) {
                    // 一次 Step 脈衝與回基準結束
                    pattern_timer = millis();
                    global_cycle++;
                    if (global_cycle >= P2_TOTAL_CYCLES) {
                        global_cycle = 0;
                        sub_step++; // 切換到下一顆馬達
                        if (sub_step >= 4) { // 四顆馬達全部測完
                            stopAllMotors();
                            current_pattern = PAT_COOLDOWN;
                            next_pattern_after_cooldown = PAT_CHIRP_SWEEP;
                            pattern_timer = millis();
                            sub_step = 0;
                            break;
                        }
                    }
                }

                setMotors(out_m1, out_m2, out_m3, out_m4);

                // 標記目前正在測哪顆馬達，方便後續大數據分類
                char step_label[16];
                sprintf(step_label, "Step_M%d", sub_step + 1);
                logTestData(step_label, out_m1, out_m2, out_m3, out_m4, 0.0f);
                break;
            }

            // ------------------------------------------------------------
            // 📊 階段四：頻率掃描正弦波 (Chirp / Sine Sweep)
            // 單顆馬達為一組，疊加振幅 P5_CHIRP_AMPLITUDE、頻率由
            // P5_FREQ_START_HZ 線性升至 P5_FREQ_END_HZ 的正弦波，藉此找出
            // 馬達+槳葉系統的機械共振點與隨頻率增加的相位延遲特性
            // （即 Bode 響應）。同樣繞 HOVER_BASELINE_DUTY 擾動，振幅有限，
            // 安全邊界與其餘階段一致。
            //
            // 線性 chirp 瞬時頻率：f(t) = f0 + (f1-f0)*t/T
            // 相位需對頻率積分：φ(t) = 2π·[ f0·t + (f1-f0)/(2T)·t² ]
            // 採用此封閉解而非逐步累加相位，避免長時間運行下的數值漂移誤差。
            // ------------------------------------------------------------
            case PAT_CHIRP_SWEEP: {
                float t = (millis() - pattern_timer) / 1000.0f;

                if (t > P5_SWEEP_DURATION_S) {
                    stopAllMotors();
                    sub_step++;
                    pattern_timer = millis();
                    if (sub_step >= 4) { // 四顆馬達全部掃完
                        current_pattern = PAT_COOLDOWN;
                        next_pattern_after_cooldown = PAT_DIFFERENTIAL_STEP;
                        sub_step = 0;
                    }
                    break;
                }

                float inst_freq = P5_FREQ_START_HZ +
                    (P5_FREQ_END_HZ - P5_FREQ_START_HZ) * (t / P5_SWEEP_DURATION_S);
                float phase = 2.0f * PI * (P5_FREQ_START_HZ * t +
                    (P5_FREQ_END_HZ - P5_FREQ_START_HZ) / (2.0f * P5_SWEEP_DURATION_S) * t * t);
                int chirp_offset = (int)roundf(P5_CHIRP_AMPLITUDE * sinf(phase));
                int swept_duty = HOVER_BASELINE_DUTY + chirp_offset;

                out_m1 = out_m2 = out_m3 = out_m4 = HOVER_BASELINE_DUTY;
                if (sub_step == 0) out_m1 = swept_duty;
                else if (sub_step == 1) out_m2 = swept_duty;
                else if (sub_step == 2) out_m3 = swept_duty;
                else if (sub_step == 3) out_m4 = swept_duty;

                setMotors(out_m1, out_m2, out_m3, out_m4);

                char chirp_label[16];
                sprintf(chirp_label, "Chirp_M%d", sub_step + 1);
                logTestData(chirp_label, out_m1, out_m2, out_m3, out_m4, inst_freq);
                break;
            }

            // ------------------------------------------------------------
            // 📊 階段五：虛擬差動階躍測試 (對角線抗衡)
            // CW 對角 (M1/M3) 與 CCW 對角 (M2/M4) 同時繞基準做反向位移：
            // 總推力不變、理論上 Roll/Pitch 也應互相抵消為零，僅在 Yaw
            // 軸上產生淨力矩——一個乾淨、解耦的純 Yaw 激勵。若 Roll/Pitch
            // 仍出現顯著偏移，代表「同一對角線內」(M1 vs M3 或 M2 vs M4)
            // 馬達特性不對稱。
            // ------------------------------------------------------------
            case PAT_DIFFERENTIAL_STEP: {
                unsigned long elapsed_diff = millis() - pattern_timer;

                // sub_step 0: 激勵正向對角 (M1/M3升, M2/M4降) 
                // sub_step 1: 激勵反向對角 (M1/M3降, M2/M4升)
                if (elapsed_diff < P3_STEP_DURATION_MS) {
                    if (sub_step == 0) {
                        out_m1 = HOVER_BASELINE_DUTY + P3_DUTY_DELTA; // CW 軸增強
                        out_m3 = HOVER_BASELINE_DUTY + P3_DUTY_DELTA;
                        out_m2 = HOVER_BASELINE_DUTY - P3_DUTY_DELTA; // CCW 軸減弱
                        out_m4 = HOVER_BASELINE_DUTY - P3_DUTY_DELTA;
                    } else {
                        out_m1 = HOVER_BASELINE_DUTY - P3_DUTY_DELTA; // CW 軸減弱
                        out_m3 = HOVER_BASELINE_DUTY - P3_DUTY_DELTA;
                        out_m2 = HOVER_BASELINE_DUTY + P3_DUTY_DELTA; // CCW 軸增強
                        out_m4 = HOVER_BASELINE_DUTY + P3_DUTY_DELTA;
                    }
                } else if (elapsed_diff >= P3_STEP_DURATION_MS * 2) {
                    // 回歸基準線冷卻結束，推進週期
                    pattern_timer = millis();
                    global_cycle++;
                    if (global_cycle >= P3_TOTAL_CYCLES) {
                        global_cycle = 0;
                        sub_step++; 
                        if (sub_step >= 2) { // 兩組對角線皆測試完畢
                            current_pattern = PAT_DONE;
                        }
                    }
                }

                setMotors(out_m1, out_m2, out_m3, out_m4);

                char diff_label[16];
                sprintf(diff_label, "Diff_Group%d", sub_step + 1);
                logTestData(diff_label, out_m1, out_m2, out_m3, out_m4, 0.0f);
                break;
            }

            // 測試全面結束
            case PAT_DONE: {
                stopAllMotors();
                digitalWrite(LED_STATUS, LOW);
                Serial.println("🏁 [五階段自動化系統辨識] 全部順利完成！");
                Serial.println("請停止 Serial Log 並將 CSV 數據導出至 Python / MATLAB。");
                current_pattern = PAT_IDLE;
                break;
            }

            default:
                break;
        }
    }
}
