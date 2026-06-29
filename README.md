# CURIO 馬達品質動態辨識測試平台 — 韌體與理論說明

> 對應程式：`CURIO_Motor_Rig_Test.ino`（搭配 `BMI088.h` / `BMI088.cpp`，遙控部分移植自 `CURIO_ELRS_Test.ino`）
> 平台：RP2350 / RP2354A
> 測試載具：四軸機架，安裝於三軸可自由旋轉的 Gimbal 測試台上
> 用途：在不拆解馬達的前提下，以低成本方式量化評估四顆空心杯馬達的製造一致性，作為馬達品管分級（QA Binning）的系統辨識工具

> **本版更新**：加入 ELRS/CRSF 遙控操作介面與 CH5 ARM 安全狀態機（見第 3 節）——通電後與 TX16S BIND、預設 DISARM，操作者扳動 CH5 開關並維持 1.5 秒後自動觸發五階段測試，且具備斷線失效保護（Failsafe）。詳細變更摘要見文末第 12 節。

---

## 1. 物理測試模型（重要：這決定了所有 Pattern 的設計邊界）

### 1.1 正確的測試條件

測試架**不是鎖固結構**：四顆馬達裝在一台完整的四軸機架上，這台機架本身被安裝在一個**三軸可自由旋轉**的 Gimbal 測試台上——只限制平移，不限制旋轉。

### 1.2 核心量測原理

若四顆「規格相同」的馬達被給予完全相同的 PWM，理論上四個推力應該互相平衡，Gimbal 上的機架不應該有明顯轉動。

> **任何「非預期的轉動」，都代表馬達彼此的實際推力/扭矩特徵不一致**（即使標稱規格相同）——這正是本測試要量化的「品質不穩」。

這裡假設馬達接頭與 MOSFET 驅動差異可忽略，量到的差異歸因於馬達本身（轉子磁力均勻度、電刷/換向器接觸品質、軸承摩擦等）。

物理映射關係沿用力矩平衡的角度來看：

- **Roll / Pitch**：對角或單邊馬達推力不一致時，機架會繞 Roll/Pitch 軸傾轉
$$\tau_{roll} \propto F_{right} - F_{left}, \qquad \tau_{pitch} \propto F_{front} - F_{rear}$$
- **Yaw**：CW 與 CCW 馬達反扭矩不平衡時，機架會繞 Yaw 軸轉動
$$\tau_{yaw} \propto \sum \tau_{CW} - \sum \tau_{CCW}$$

由於是**自由旋轉**而非鎖固量測應變，這個方法其實比剛性結構更敏感——只要 Gimbal 軸承摩擦夠低，極小的持續力矩不平衡就會被積分成可觀察的角度偏移，原理上類似力矩平衡秤（torque balance），比直接量應變/反作用力更靈敏。

### 1.3 因此衍生的安全設計準則

Gimbal 是自由旋轉，**沒有像鎖固結構那樣的剛性回正力**。任何測試 Pattern 都**不能**讓單顆馬達從 0 直接跳到大 Duty、其餘三顆完全停轉——這樣只有一顆馬達在推、零抗衡，會讓機架產生劇烈、大角度甚至失控的轉動，不只破壞「小信號線性響應」的量測假設，也有撞擊 Gimbal 機構極限的風險。

**統一設計原則**：除了 Pattern 1（三角波，四顆同步），其餘所有 Pattern 都採用「**繞工作點做小信號擾動**」：四顆馬達先一起穩定在共用基準 `HOVER_BASELINE_DUTY`（模擬接近懸停的工作點），再對單顆（或對角成對）馬達疊加一個侷限在安全範圍內的位移量。瞬時不平衡力矩永遠被限制在這個位移量等級，機架只會小幅、可預期、可回復地傾轉。

| 全域安全參數 | 數值 | Duty % |
|---|---|---|
| `HOVER_BASELINE_DUTY`（共用基準） | 76 | ≈ 30% |
| 各階段最大位移量 | 30 ~ 40 LSB | ≈ 12% ~ 16% |
| 任何單顆馬達瞬時最大 Duty | 116 | ≈ 45.5% |

---

## 2. 硬體與韌體配置

### 2.1 接腳定義

| 訊號 | 接腳 | 說明 |
|---|---|---|
| M1 | GPIO 0 | 右前馬達，CW |
| M2 | GPIO 11 | 右後馬達，CCW |
| M3 | GPIO 14 | 左後馬達，CW |
| M4 | GPIO 28 | 左前馬達，CCW |
| LED_STATUS | GPIO 7 | 測試執行中指示燈 |
| I2C0_SDA | GPIO 20 | BMI088 資料線 |
| I2C0_SCL | GPIO 25 | BMI088 時鐘線 |
| ELRS TX1 | GPIO 4 | CURIO UART1 → ELRS 接收機 RX |
| ELRS RX1 | GPIO 5 | CURIO UART1 ← ELRS 接收機 TX |

### 2.2 感測器配置

```
BMI088_ACC_ADDR  = 0x18   // ALT 位址（非函式庫預設的 0x19）
BMI088_GYRO_ADDR = 0x69   // 預設位址
```

⚠️ 加速度計用的是 **ALT 位址 0x18**，請確認測試架上 BMI088 的 ADDR 腳位接線與此一致。I²C Clock 為 400 kHz（Fast Mode）。

### 2.3 PWM 驅動參數

```
PWM_FREQ  = 20 kHz   // 高於人耳敏感頻段
PWM_RANGE = 255      // 8-bit 解析度，1 LSB ≈ 0.392% Duty
```

### 2.4 姿態估算（Mahony AHRS）

僅使用加速度計做傾角融合（無磁力計），`kp=2.0 / ki=0.005`。`roll`、`pitch` 為融合後角度（度）；`gx/gy/gz` 維持為原始角速度（記錄時換算為 deg/s）。Yaw 沒有絕對角度參考，因此分析時一律用 `GyroZ`（角速度）而非積分角度。

### 2.5 開機自動校正

`setup()` 執行 500 次取樣（約 1.5 秒）的靜態校正，且只扣除「相對於標準重力的偏差量」，不是整個重力分量——校正後若水平靜置，`az` 應仍接近 $9.80665\ \text{m/s}^2$，可作健康檢查基準。

### 2.6 韌體層級優化（與物理模型修正獨立的程式碼品質改進）

| 項目 | 修改前 | 修改後 | 理由 |
|---|---|---|---|
| Serial 日誌輸出 | 每行 11 次 `Serial.print()` | `snprintf()` 組好整行字串後單次 `Serial.write()` | 11 次零散呼叫與浮點轉字串開銷，會侵蝕 500Hz 節拍的時間預算，造成取樣間隔抖動（jitter） |
| 序列埠指令解析 | `String cmd = Serial.readStringUntil('\n')` | 固定長度 `char` buffer 逐位元組非阻塞收集 | `readStringUntil` 在換行符未到齊時會依預設逾時（1000ms）整個阻塞 `loop()`，凍結 IMU/Mahony/PWM 更新 |
| 加速度計取樣 | 每次 `loop()` 都重新讀取 | 依實際 ODR_100（10ms）節流讀取 | 多數時間讀到的是尚未更新的舊值，浪費 I2C 匯流排時間 |
| 馬達輸出 | 各 Pattern 各自重複 4 行 `analogWrite()` | 統一 `setMotors()`，內部用 `constrain()` 保險 | 去重複，且日後調參數造成數值溢出時不會送出超出 0–255 範圍的 Duty |
| `logTestData()` 的 `freqHz` 參數 | （新增功能） | **刻意不用預設參數**，呼叫端一律明確傳值 | Arduino IDE 的 ctags 自動原型生成器，若函式定義帶有預設參數值，產生的原型會重複同一個預設值，導致「重複預設參數」編譯錯誤 |

---

## 3. ELRS / CRSF 遙控與 ARM 安全機制（本版新增）

### 3.1 操作流程

對應 TX16S 實機操作的完整時序：

```
① 通電
   └─▶ CURIO 與 TX16S 完成 ELRS BIND，開始收到 CRSF 訊框，CH5 應為低位 → DISARMED
② 操作者把 TX16S 的 CH5 開關撥到高位
   └─▶ 進入 ARMING，開始倒數 1.5 秒（ARM_HOLD_MS）
       ├─ 倒數中提前放手（CH5 回到低位）→ 取消，回到 DISARMED
       └─ 倒數滿 1.5 秒仍維持高位 → ARMED，自動觸發五階段測試（等同手動輸入 'START'）
③ 測試運行中
   ├─ CH5 回到低位 → 立即停機，回到 DISARMED
   ├─ ELRS 訊號中斷 ≥ 500ms → 視為等同 CH5 低位，立即停機（Failsafe）
   └─ 五階段測試自然跑完 → 回到 DISARMED，且要求重新扳動開關才能再次觸發
```

這個 ARM 狀態機是獨立於測試狀態機（`current_pattern`）**之上的一層安全閘門**：`current_pattern` 只決定「測試跑到哪個階段」，`arm_state` 才決定「現在到底有沒有被允許讓馬達轉」。即使測試正跑到 Chirp Sweep 或 Differential Step 的任何一個瞬間，CH5 低位都會無視目前進度直接強制停機。

### 3.2 CRSF 協議解析

直接移植 `CURIO_ELRS_Test.ino` 中已驗證的解析邏輯（CRC8 校驗、11-bit 通道解碼、`0xC8` 同步位元組、`0x16` 通道訊框類型），完全相同的實作，沒有重新發明：

```
ELRS_SERIAL = Serial2,  420000 baud
PIN_ELRS_TX = GPIO4,    PIN_ELRS_RX = GPIO5
channels[16]，0-indexed；CH5（Arm 開關）對應 channels[4]
```

`parseCRSF()` 在每次 `loop()` 都會被無條件呼叫（不受 2ms 狀態機節流影響），持續把 UART RX 緩衝區排空、組訊框、驗證 CRC、解碼——這點很重要：CRSF 在 420000 baud 下訊框到達頻率可能遠高於 2ms 一次的測試節拍，若被節流卡住，硬體 UART 緩衝區會在下一個訊框抵達前溢位。

### 3.3 ARM 三態狀態機

```cpp
enum ArmState { ARM_DISARMED, ARM_ARMING, ARM_ARMED };
```

| 安全參數 | 數值 | 說明 |
|---|---|---|
| `CH5_ARM_THRESHOLD_US` | 1700 us | 高於此值視為「開關切高」 |
| `CH5_DISARM_THRESHOLD_US` | 1300 us | 低於此值視為「開關切低」 |
| （兩者間 400us 死區） | — | 避免開關邊界訊號抖動造成誤判 |
| `ARM_HOLD_MS` | 1500 ms | 切高後需維持的確認時間（落在題目要求的 1~2 秒區間中點） |
| `CRSF_LINK_TIMEOUT_MS` | 500 ms | 超過此時間沒收到新 CRSF 訊框，視為斷線（Failsafe 判定依據） |

狀態轉移：

```
DISARMED ──CH5切高且非「需重新扳動」──▶ ARMING ──倒數滿1.5秒──▶ ARMED
   ▲                                      │ CH5提前切低                │
   └──────────────────────────────────────┘                            │
   ▲                                                                    │
   └────────── CH5切低 或 斷線≥500ms：硬中斷，立即停機 ───────────────────┤
   ▲                                                                    │
   └────────── 測試自然跑完：標記 ch5_require_cycle=true ─────────────────┘
```

**為什麼需要 `ch5_require_cycle`？** 如果測試自然跑完（或被 Serial `STOP` 中止）時操作者的手還沒放開 CH5 開關（仍停在高位），沒有這個旗標的話，下一輪迴圈會立刻偵測到「CH5 高位」而重新進入 ARMING，1.5 秒後又自動重新觸發一次完整 20 分鐘測試——這顯然不是預期行為。因此設計上要求：每次測試結束（無論自然完成或被中止）後，必須先看到 CH5 回到低位一次，才解除這個旗標、允許下一次 ARM。

### 3.4 Serial 與遙控器雙控制源的仲裁規則

| 情境 | 系統反應 |
|---|---|
| 已偵測到 ELRS 連線時輸入 Serial `START` | **被拒絕**，提示改用 TX16S CH5 開關 |
| 未連接收發機（無 ELRS 連線）時輸入 Serial `START` | 正常啟動（保留無遙控器時的單機桌面測試彈性） |
| 任何時候輸入 Serial `STOP` | 永遠有效，立即停機，並同步重置 `arm_state`、要求重新扳動 CH5 |

設計理由：一旦確認遙控器已連線，ARM 的權責就應該完全交給實體開關，避免「鍵盤」與「遙控器」兩個控制源同時都能讓馬達轉動而造成混淆或競態。但 `STOP` 作為緊急停機指令，不論連線狀態都必須無條件生效。

### 3.5 待機狀態監控輸出

只有在 `current_pattern == PAT_IDLE`（尚未開始測試）時，韌體才會每 0.5 秒印一行狀態：

```
[ELRS] Link=OK  CH5=1003us  ArmState=DISARMED
```

一旦測試開始（CSV 表頭已印出），這個監控輸出會自動停止，避免汙染 CSV 資料流。

---

## 4. 自動化測試狀態機

```
IDLE ──ARM/START──▶ TRIANGLE ──(300 cycles)──▶ COOLDOWN ──▶ DEADZONE_SWEEP ──(4 motors)──▶ COOLDOWN
                                                                                                   │
                                                                                                   ▼
                                                                 CHIRP_SWEEP ◀──COOLDOWN◀── STEP_RESPONSE
                                                                     │
                                                                     ▼
                                                                 COOLDOWN ──▶ DIFFERENTIAL_STEP ──▶ DONE ──▶ IDLE
```

冷卻階段採用**單一** `PAT_COOLDOWN` + 全域變數 `next_pattern_after_cooldown`：每次進冷卻前先指定「冷卻完後要去哪一站」，新增測試階段時不需要再增加列舉值。

主迴圈以 `now_ms - last_loop_time >= 2` 嚴格鎖定 **500 Hz（2 ms）** 步進執行測試狀態機；ELRS 解析與 ARM 狀態機則不受此節流限制，每次 `loop()` 都會執行。

---

## 5. 五階段測試模式詳解

### 5.1 階段一：交錯三角波（Triangle Wave）—— 全幅油門範圍的同步性檢驗

四顆馬達**同步追蹤同一個目標值**（round-robin 寫入造成天生 2ms 級相位錯位，作為額外的高頻差分激勵層），目標值以 $0\% \to 50\% \to 0\%$ 三角波變化。這是對「同 PWM 是否同推力」這個核心假設，在**整個油門範圍**內做一次全面掃描。

| 參數 | 數值 |
|---|---|
| `P1_DUTY_MAX` | 128（≈50.2%） |
| 單次三角波週期 | 2048 ms |
| `P1_TOTAL_CYCLES` | 300 |
| **本階段時長** | **≈ 614.4 s（10 分 14 秒）** |

---

### 5.2 階段二：低幅度漸增死區掃描（Dead-zone Sweep）

**目的**：找出每顆馬達「相對於共同基準的增量響應門檻」——換向器/軸承靜摩擦造成的死區大小。

**設計**：四顆馬達先一起停在 `HOVER_BASELINE_DUTY`，再對其中一顆每 200ms 緩慢疊加 1 LSB，一路爬升到 `+P4_SWEEP_MAX_DELTA`（40 LSB ≈ 15.7%），其餘三顆全程維持基準不動。

```cpp
out_m1 = out_m2 = out_m3 = out_m4 = HOVER_BASELINE_DUTY;
int swept_duty = HOVER_BASELINE_DUTY + delta;   // delta 每 200ms 增加 1
```

| 參數 | 數值 |
|---|---|
| `P4_SWEEP_MAX_DELTA` | 40 LSB（≈15.7%） |
| `P4_STEP_INTERVAL_MS` | 200 ms / LSB |
| 單顆馬達掃描時長 | ≈ 8.2 s |
| **本階段時長** | **≈ 32.8 s** |

**分析重點**：以 `Mx` 欄位（即 baseline+delta）對 `GyroX/Y/Z` 標準差或閾值偏移畫圖，找出該馬達「開始可辨識偏離噪音底」的 delta 值，即為該馬達的增量死區。

---

### 5.3 階段三：單馬達小信號階躍響應（Step Response）

四顆馬達先一起停在 `HOVER_BASELINE_DUTY`，待測馬達（`sub_step` 0~3 對應 M1~M4）疊加 `+P2_DUTY_DELTA`（38 LSB ≈15%）維持半個週期，再回到基準，其餘三顆全程維持基準。瞬時不平衡力矩被限制在 38 LSB 範圍內，避免單顆全幅推力、零抗衡造成機架劇烈轉動。

```cpp
out_m1 = out_m2 = out_m3 = out_m4 = HOVER_BASELINE_DUTY;
if (elapsed_step < P2_STEP_DURATION_MS) {
    // 僅待測馬達疊加 +P2_DUTY_DELTA，其餘維持基準
}
```

每個週期內同時包含一個「上升緣」與一個「下降緣」，可分別擬合上升/下降時間常數。

| 參數 | 數值 |
|---|---|
| `P2_DUTY_DELTA` | 38 LSB（≈15%） |
| 脈衝段 / 回基準段 | 800 ms / 800 ms |
| `P2_TOTAL_CYCLES` | 50（每顆馬達） |
| **本階段時長** | **≈ 320 s（5 分 20 秒）** |

**分析重點**：以一階系統模型擬合響應曲線
$$y(t) = y_0 + K\left(1 - e^{-t/\tau}\right)$$
分別對上升緣與下降緣擬合 $\tau$，比較四顆馬達的時間常數。

---

### 5.4 階段四：頻率掃描正弦波（Chirp / Sine Sweep）

**目的**：找出馬達+槳葉系統的機械共振點，以及隨頻率增加而成長的相位延遲（Bode 響應）。

$$f(t) = f_0 + (f_1-f_0)\frac{t}{T}, \qquad \phi(t) = 2\pi\left[f_0 t + \frac{f_1-f_0}{2T}t^2\right]$$
$$duty(t) = \text{baseline} + A\sin(\phi(t))$$

韌體用這個**封閉解**直接計算相位，而不是逐步累加 $\phi \mathrel{+}= 2\pi f(t)\,dt$，避免長時間運行下的浮點誤差累積（相位漂移）。

```cpp
float phase = 2.0f * PI * (P5_FREQ_START_HZ * t +
              (P5_FREQ_END_HZ - P5_FREQ_START_HZ) / (2.0f * P5_SWEEP_DURATION_S) * t * t);
int chirp_offset = (int)roundf(P5_CHIRP_AMPLITUDE * sinf(phase));
```

| 參數 | 數值 |
|---|---|
| 振幅 `P5_CHIRP_AMPLITUDE` | 30 LSB（≈11.8%） |
| 頻率範圍 | 0.5 Hz → 15 Hz |
| 單顆馬達掃描時長 | 20 s |
| **本階段時長** | **≈ 80 s（1 分 20 秒）** |

頻率上限 15Hz 遠低於狀態機 500Hz 取樣的 Nyquist 極限（250Hz），不會有混疊問題。CSV 中每一筆資料同時記錄該瞬間的 `FreqHz`，供事後依頻率分箱分析。

**分析重點**：將 `GyroX/Y/Z` 依 `FreqHz` 分箱，用最小平方正弦擬合（或短窗 FFT）抓出每個頻率下的響應振幅與相位延遲，畫出 Bode 圖。

---

### 5.5 階段五：虛擬差動階躍（Differential Step）—— 對角解耦

CW 對角（M1/M3）與 CCW 對角（M2/M4）同時繞基準做反向位移：

```cpp
out_m1 = out_m3 = HOVER_BASELINE_DUTY + P3_DUTY_DELTA;  // CW 對角
out_m2 = out_m4 = HOVER_BASELINE_DUTY - P3_DUTY_DELTA;  // CCW 對角
```

理論推導（X 型機架，FR=M1, RR=M2, RL=M3, FL=M4）：

$$\text{Roll} \propto (M1{+}M2) - (M3{+}M4) = 0, \qquad \text{Pitch} \propto (M1{+}M4)-(M2{+}M3) = 0$$
$$\text{Yaw} \propto 2(baseline{+}\Delta) - 2(baseline{-}\Delta) = 4\Delta$$

總推力不變、Roll/Pitch 理論上互相抵消為零，**僅在 Yaw 軸產生淨力矩**。若實測 Roll/Pitch 仍出現顯著偏移，代表「**同一對角線內**」（M1 vs M3，或 M2 vs M4）馬達特性不對稱。

| 參數 | 數值 |
|---|---|
| `P3_DUTY_DELTA` | 38 LSB（≈15%） |
| 衝擊段 / 回歸段 | 600 ms / 600 ms |
| `P3_TOTAL_CYCLES` | 50（每組對角線） |
| **本階段時長** | **≈ 120 s（2 分鐘）** |

---

### 5.6 過渡冷卻階段（Cooldown）

每兩個測試階段之間插入 **10 秒**冷卻：前 8 秒全馬達歸零靜止，後 2 秒持續執行 `resetMahonyFilter()` 強制清空四元數與陀螺儀偏置積分項。

---

## 6. 操作流程

1. 將測試架（連同四軸機架）安裝在 Gimbal 上，確保可自由旋轉、無線材纏繞限制行程，螺旋槳鎖固牢靠。
2. 上電：CURIO 與 TX16S 完成 ELRS BIND（CH5 應為低位），同時等待 BMI088 連線與 500 樣本靜態校正（約 1.5 秒）。
3. 確認序列埠（115200 baud）顯示「✅ 基準校正完成」，且 `[ELRS] Link=OK ArmState=DISARMED`。
4. 將 TX16S 的 CH5 開關撥到高位並維持，約 1.5 秒後韌體自動 ARMED、開始五階段測試
   （若未連接收發機，可直接輸入 Serial `START` 作為單機測試替代方案）。
5. 全程約 **20 分鐘**（詳細 breakdown 見第 7 節），期間勿觸碰機架。CH5 切回低位或 ELRS 斷線可隨時安全中止；Serial `STOP` 在任何時候都能緊急中止。
6. 完成後印出「🏁 全部順利完成」，將序列埠輸出存成 `.csv`。下一輪測試前須將 CH5 重新扳低再扳高一次。

---

## 7. 總時長明細

| 階段 | 時長 |
|---|---|
| Triangle | 614.4 s |
| Cooldown | 10 s |
| Deadzone Sweep | 32.8 s |
| Cooldown | 10 s |
| Step Response | 320 s |
| Cooldown | 10 s |
| Chirp Sweep | 80 s |
| Cooldown | 10 s |
| Differential Step | 120 s |
| **總計** | **≈ 1207 s ≈ 20 分 7 秒** |

---

## 8. 資料格式

```
Pattern,Cycle,M1,M2,M3,M4,Roll,Pitch,GyroX,GyroY,GyroZ,FreqHz
```

| 欄位 | 單位 | 說明 |
|---|---|---|
| Pattern | — | `Triangle` / `Deadzone_M1~M4` / `Cooldown` / `Step_M1~M4` / `Chirp_M1~M4` / `Diff_Group1~2` |
| Cycle | — | 僅 Triangle / Step / Diff 有意義；Deadzone / Chirp 不使用此欄位（恆為 0） |
| M1–M4 | 0–255 | 各馬達當下 PWM Duty 值 |
| Roll, Pitch | deg | Mahony 解算姿態角 |
| GyroX/Y/Z | deg/s | 原始角速度（已扣除靜態偏置） |
| FreqHz | Hz | 僅 `Chirp_Mx` 列為非零值，記錄該瞬間的 Chirp 瞬時頻率 $f(t)$ |

資料清洗第一步：剔除 `Cooldown` 標籤的列，再依 `Pattern` 欄位切分後分別處理。待機階段的 `[ELRS]` 狀態監控屬於人類可讀的提示訊息，非 CSV 格式，分析時忽略即可。

---

## 9. 數據分析方法總表

| 階段 | 繪圖方式 | 判讀依據 |
|---|---|---|
| Triangle | Duty vs GyroZ | 全油門範圍內的推力/反扭矩不對稱 |
| Triangle | Duty vs Gyro 絕對值（升/降段分開） | 遲滯環面積 → 動態響應快慢 |
| Deadzone Sweep | Δduty vs Gyro 偏離噪音底的閾值 | 各馬達增量死區大小比較 |
| Step Response | 階躍上升/下降緣擬合 $\tau$ | 各馬達動態時間常數比較 |
| Chirp Sweep | 依 FreqHz 分箱做 Bode（振幅+相位 vs 頻率） | 機械共振點、頻寬比較 |
| Differential Step | Group1 vs Group2 的 Roll/Pitch 峰值 | 對角線內 $K_t$ 不對稱 |

---

## 10. 已實作功能總覽

| 功能 | 狀態 |
|---|---|
| 三角波交錯激勵 | ✅ |
| 低幅度漸增死區掃描 | ✅ |
| 單馬達小信號階躍響應 | ✅ |
| 頻率掃描正弦波（Chirp） | ✅ |
| 虛擬差動正負階躍 | ✅ |
| ELRS/CRSF 遙控解析 | ✅（本版新增） |
| CH5 ARM/DISARM 安全狀態機 | ✅（本版新增） |
| ELRS 斷線 Failsafe | ✅（本版新增） |

---

## 11. 操作安全注意事項

- 確認 Gimbal 三軸旋轉行程內無線材纏繞、無障礙物；所有測試 Pattern 皆設計為「繞基準小信號擾動」，正常情況下機架只會小幅可控傾轉。若觀察到劇烈或持續加速的轉動，立即將 CH5 切回低位（或輸入 Serial `STOP`）並檢查機構與接線。
- LED（GPIO 7）點亮代表馬達已解鎖且狀態機正在運行，靠近機架前務必先確認燈號熄滅、CH5 已切低，或已送出 `STOP`。
- ELRS 訊號中斷 ≥500ms 會自動視為失效保護（Failsafe）並停機，但仍建議測試時保持操作者與發射機在視距內，不依賴 Failsafe 作為唯一安全手段。
- 已連線 ELRS 時，Serial `START` 會被拒絕——這是刻意設計，避免雙重控制源同時可以讓馬達轉動。
- 若中途異常，CH5 切低或輸入 `STOP` 皆可安全終止，不需要重新上電；但下一輪測試前必須讓 CH5 經過一次低位才能再次 ARM。

---

## 12. 版本變更摘要

**v3（本版）**
1. **新增 ELRS/CRSF 遙控解析**：移植 `CURIO_ELRS_Test.ino` 的協議解析邏輯（第 3.2 節）。
2. **新增 CH5 ARM 安全狀態機**：DISARMED → ARMING（1.5 秒確認）→ ARMED，自動觸發測試（第 3.3 節）。
3. **新增 Failsafe**：ELRS 斷線 ≥500ms 等同 CH5 低位，立即停機。
4. **Serial 與遙控器雙控制源仲裁**：已連線時 Serial `START` 停用，`STOP` 永遠有效（第 3.4 節）。
5. **新增待機狀態監控輸出**：僅 `PAT_IDLE` 時印出連線/CH5/ARM 狀態，不干擾 CSV 資料流（第 3.5 節）。
6. 全文章節重新編號以容納新增的第 3 節。

**v2**
1. **物理模型修正**：測試架是三軸自由旋轉 Gimbal，不是鎖固結構。
2. **Pattern 2 重新設計**：階躍響應改為「繞 `HOVER_BASELINE_DUTY` 基準的小信號擾動」。
3. **新增 Pattern**：低幅度漸增死區掃描、頻率掃描正弦波 Chirp，CSV 新增 `FreqHz` 欄位。
4. **狀態機重構**：`PAT_COOLDOWN_1/2` 合併為單一 `PAT_COOLDOWN` + `next_pattern_after_cooldown`。
5. **韌體時間精度優化**：Serial 緩衝寫入、非阻塞指令解析、加速度計取樣節流、統一 `setMotors()` 含 `constrain()` 保險。
6. **工具鏈陷阱規避**：`logTestData()` 刻意不使用預設參數。
