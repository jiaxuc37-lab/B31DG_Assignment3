#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// --- Hardware Pin Definitions ---
#define DEV_SYNC_PIN    4
#define DEV_IN_A        13
#define DEV_IN_B        14
#define DEV_IN_S        27
#define DEV_MODE_PIN    26

#define ACK_A_PIN       16
#define ACK_B_PIN       17
#define ACK_AGG_PIN     5
#define ACK_C_PIN       18
#define ACK_D_PIN       19
#define ACK_S_PIN       21

// --- Core Scheduling Parameters ---
#define CORE_ID         1
#define REPORT_LIMIT_S  2
#define TICK_10MS       pdMS_TO_TICKS(10)
#define TICK_20MS       pdMS_TO_TICKS(20)
#define TICK_50MS       pdMS_TO_TICKS(50)

#if BOARD_PROFILE == 2 // 160MHz (ESP32-C3)
  #define CYC_A 448000u
  #define CYC_B 640000u
  #define CYC_G 320000u
  #define CYC_C 1120000u
  #define CYC_D 640000u
  #define CYC_S 400000u
#else // 240MHz (ESP32-Wroom)
  #define CYC_A 672000u
  #define CYC_B 960000u
  #define CYC_G 480000u
  #define CYC_C 1680000u
  #define CYC_D 960000u
  #define CYC_S 600000u
#endif

// --- Data Structure for Monitoring ---
struct TaskMetrics {
    uint32_t count;
    uint32_t fail_cnt;
    uint64_t last_start;
    uint64_t rel_base;
    uint64_t peak_exe;
    int64_t peak_late;
    uint64_t limit_us;
};

// --- Global Context and State ---
static struct {
    TaskMetrics m_a, m_b, m_agg, m_c, m_d, m_s;
    uint64_t t_zero;
    bool is_finalized;
    uint32_t s_queue[32];
    uint32_t s_head, s_tail, s_size;
} sys_ctx;

static volatile uint32_t pulse_a = 0, pulse_b = 0;
static volatile bool sys_active = false;
static volatile bool stop_signal = false;
static TickType_t base_tick = 0;

static uint32_t id_a=0, id_b=0, id_agg=0, id_c=0, id_d=0, id_s=0;
static uint32_t tok_a=0, tok_b=0;
static bool pub_a=false, pub_b=false;

static portMUX_TYPE pulse_lock = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t mtx_tok, mtx_log, sem_sporadic, mtx_stat;

// --- Utility Functions ---
static inline uint32_t read_ccount() {
    uint32_t c;
    asm volatile("rsr %0, ccount" : "=a"(c));
    return c;
}

static uint32_t heavy_compute(uint32_t target_cycles, uint32_t seed) {
    uint32_t ts = read_ccount();
    uint32_t out = seed ^ 0xDEADBEEFu;
    while ((read_ccount() - ts) < target_cycles) {
        out = (out ^ (out >> 15)) * 0x45D9F3Bu;
        out = (out ^ (out >> 13)) * 0x45D9F3Bu;
        out ^= (out >> 16);
        asm volatile("" ::: "memory");
    }
    return out;
}

void print_log(const char* tag, uint32_t id, uint32_t val1, uint32_t val2 = 0) {
    xSemaphoreTake(mtx_log, portMAX_DELAY);
    if(tag[0] == 'C' || tag[0] == 'D' || tag[0] == 'S') 
        Serial.printf("%s,%u,%u\n", tag, id, val1);
    else 
        Serial.printf("%s,%u,%u,%u\n", tag, id, val1, val2);
    xSemaphoreGive(mtx_log);
}

// --- Monitoring and Metrics Logic ---
void mark_begin(TaskMetrics &m, uint32_t id, uint64_t rel_off) {
    xSemaphoreTake(mtx_stat, portMAX_DELAY);
    m.last_start = micros();
    m.rel_base = sys_ctx.t_zero + rel_off;
    xSemaphoreGive(mtx_stat);
}

void mark_end(TaskMetrics &m) {
    uint64_t now = micros();
    xSemaphoreTake(mtx_stat, portMAX_DELAY);
    uint64_t dur = now - m.last_start;
    int64_t lateness = (int64_t)now - (int64_t)(m.rel_base + m.limit_us);
    if (dur > m.peak_exe) m.peak_exe = dur;
    if (lateness > m.peak_late) m.peak_late = lateness;
    m.count++;
    if (lateness > 0) m.fail_cnt++;
    xSemaphoreGive(mtx_stat);
}

// --- Interrupt Service Routines (ISRs) ---
void IRAM_ATTR isr_sync() {
    if (sys_active) return;
    sys_ctx.t_zero = micros();
    base_tick = xTaskGetTickCountFromISR();
    sys_active = true;
    
    // Release all tasks waiting for the SYNC signal
    BaseType_t hi = pdFALSE;
    vTaskNotifyGiveFromISR(xTaskGetHandle("TaskA"), &hi);
    vTaskNotifyGiveFromISR(xTaskGetHandle("TaskB"), &hi);
    vTaskNotifyGiveFromISR(xTaskGetHandle("TaskG"), &hi);
    vTaskNotifyGiveFromISR(xTaskGetHandle("TaskC"), &hi);
    vTaskNotifyGiveFromISR(xTaskGetHandle("TaskD"), &hi);
    vTaskNotifyGiveFromISR(xTaskGetHandle("TaskS"), &hi);
    portYIELD_FROM_ISR(hi);
}

void IRAM_ATTR isr_a() { if(sys_active) pulse_a++; }
void IRAM_ATTR isr_b() { if(sys_active) pulse_b++; }
void IRAM_ATTR isr_s() {
    if(!sys_active) return;
    BaseType_t hi = pdFALSE;
    xSemaphoreGiveFromISR(sem_sporadic, &hi);
    portYIELD_FROM_ISR(hi);
}

// --- Task Implementations ---

void worker_a(void* p) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    TickType_t next = base_tick;
    sys_ctx.m_a.limit_us = 10000; // 10ms deadline
    while (!stop_signal) {
        portENTER_CRITICAL(&pulse_lock);
        uint32_t c = pulse_a; pulse_a = 0;
        portEXIT_CRITICAL(&pulse_lock);

        mark_begin(sys_ctx.m_a, id_a, (uint64_t)id_a * 10000);
        digitalWrite(ACK_A_PIN, HIGH);
        uint32_t r = heavy_compute(CYC_A, id_a ^ c);
        digitalWrite(ACK_A_PIN, LOW);
        mark_end(sys_ctx.m_a);

        xSemaphoreTake(mtx_tok, portMAX_DELAY);
        tok_a = r; pub_a = true;
        xSemaphoreGive(mtx_tok);
        
        print_log("A", id_a++, c, r);
        vTaskDelayUntil(&next, TICK_10MS);
    }
    vTaskDelete(NULL);
}

void worker_b(void* p) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    TickType_t next = base_tick;
    sys_ctx.m_b.limit_us = 20000; // 20ms deadline
    while (!stop_signal) {
        portENTER_CRITICAL(&pulse_lock);
        uint32_t c = pulse_b; pulse_b = 0;
        portEXIT_CRITICAL(&pulse_lock);

        mark_begin(sys_ctx.m_b, id_b, (uint64_t)id_b * 20000);
        digitalWrite(ACK_B_PIN, HIGH);
        uint32_t r = heavy_compute(CYC_B, id_b ^ c);
        digitalWrite(ACK_B_PIN, LOW);
        mark_end(sys_ctx.m_b);

        xSemaphoreTake(mtx_tok, portMAX_DELAY);
        tok_b = r; pub_b = true;
        xSemaphoreGive(mtx_tok);

        print_log("B", id_b++, c, r);
        vTaskDelayUntil(&next, TICK_20MS);
    }
    vTaskDelete(NULL);
}

void worker_agg(void* p) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    TickType_t next = base_tick;
    sys_ctx.m_agg.limit_us = 20000;
    while (!stop_signal) {
        uint32_t combined = 0;
        xSemaphoreTake(mtx_tok, portMAX_DELAY);
        combined = (pub_a && pub_b) ? (tok_a ^ tok_b) : 0xDEADBEEF;
        xSemaphoreGive(mtx_tok);

        mark_begin(sys_ctx.m_agg, id_agg, (uint64_t)id_agg * 20000);
        digitalWrite(ACK_AGG_PIN, HIGH);
        uint32_t r = heavy_compute(CYC_G, combined ^ 0xD4);
        digitalWrite(ACK_AGG_PIN, LOW);
        mark_end(sys_ctx.m_agg);

        print_log("AGG", id_agg++, combined, r);
        vTaskDelayUntil(&next, TICK_20MS);
    }
    vTaskDelete(NULL);
}

void worker_cd(void* p) {
    bool is_task_c = ((int)p == 'C');
    TaskMetrics &m = is_task_c ? sys_ctx.m_c : sys_ctx.m_d;
    uint32_t &id = is_task_c ? id_c : id_d;
    int pin = is_task_c ? ACK_C_PIN : ACK_D_PIN;
    uint32_t cycles = is_task_c ? CYC_C : CYC_D;
    
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    TickType_t next = base_tick;
    m.limit_us = 50000; // 50ms deadline

    while (!stop_signal) {
        // Execute only if DEV_MODE_PIN is HIGH (FreeRTOS mode behavior)
        if (digitalRead(DEV_MODE_PIN) == HIGH) {
            mark_begin(m, id, (uint64_t)id * 50000);
            digitalWrite(pin, HIGH);
            uint32_t r = heavy_compute(cycles, id);
            digitalWrite(pin, LOW);
            mark_end(m);
            print_log(is_task_c ? "C" : "D", id++, r);
        }
        vTaskDelayUntil(&next, TICK_50MS);
    }
    vTaskDelete(NULL);
}

void worker_s(void* p) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    sys_ctx.m_s.limit_us = 30000; // 30ms response time requirement
    while (!stop_signal) {
        if (xSemaphoreTake(sem_sporadic, portMAX_DELAY) == pdTRUE) {
            if (stop_signal) break;
            mark_begin(sys_ctx.m_s, id_s, micros() - sys_ctx.t_zero);
            digitalWrite(ACK_S_PIN, HIGH);
            uint32_t r = heavy_compute(CYC_S, id_s);
            digitalWrite(ACK_S_PIN, LOW);
            mark_end(sys_ctx.m_s);
            print_log("S", id_s++, r);
        }
    }
    vTaskDelete(NULL);
}

// --- Initialization and Final Reporting ---

void setup() {
    Serial.begin(115200);
    
    // Resource Initialization
    mtx_tok = xSemaphoreCreateMutex();
    mtx_log = xSemaphoreCreateMutex();
    mtx_stat = xSemaphoreCreateMutex();
    sem_sporadic = xSemaphoreCreateCounting(20, 0);

    // Pin Configuration
    pinMode(DEV_SYNC_PIN, INPUT_PULLDOWN);
    pinMode(DEV_IN_A, INPUT);
    pinMode(DEV_IN_B, INPUT);
    pinMode(DEV_IN_S, INPUT_PULLDOWN);
    pinMode(DEV_MODE_PIN, INPUT_PULLDOWN);
    
    int outs[] = {ACK_A_PIN, ACK_B_PIN, ACK_AGG_PIN, ACK_C_PIN, ACK_D_PIN, ACK_S_PIN};
    for(int i=0; i<6; i++) { 
        pinMode(outs[i], OUTPUT); 
        digitalWrite(outs[i], LOW); 
    }

    // Task Creation (Pinned to a single core as per requirements)
    xTaskCreatePinnedToCore(worker_a, "TaskA", 3072, NULL, 5, NULL, CORE_ID);
    xTaskCreatePinnedToCore(worker_b, "TaskB", 3072, NULL, 4, NULL, CORE_ID);
    xTaskCreatePinnedToCore(worker_agg, "TaskG", 3072, NULL, 3, NULL, CORE_ID);
    xTaskCreatePinnedToCore(worker_cd, "TaskC", 3072, (void*)'C', 1, NULL, CORE_ID);
    xTaskCreatePinnedToCore(worker_cd, "TaskD", 3072, (void*)'D', 1, NULL, CORE_ID);
    xTaskCreatePinnedToCore(worker_s, "TaskS", 3072, NULL, 2, NULL, CORE_ID);

    // Interrupt Attachment
    attachInterrupt(DEV_SYNC_PIN, isr_sync, RISING);
    attachInterrupt(DEV_IN_A, isr_a, RISING);
    attachInterrupt(DEV_IN_B, isr_b, RISING);
    attachInterrupt(DEV_IN_S, isr_s, RISING);
}

void loop() {
    if (sys_active && !sys_ctx.is_finalized) {
        // Stop execution and report after the time limit
        if (micros() - sys_ctx.t_zero > (uint64_t)REPORT_LIMIT_S * 1000000) {
            stop_signal = true;
            sys_ctx.is_finalized = true;
            
            Serial.println("FINAL_REPORT_BEGIN");
            auto r = [](const char* n, TaskMetrics &m) {
                Serial.printf("[MON] %s jobs=%u misses=%u max_exec=%lluus worst_late=%lldus\n", 
                               n, m.count, m.fail_cnt, m.peak_exe, m.peak_late);
            };
            r("A", sys_ctx.m_a); r("B", sys_ctx.m_b); r("AGG", sys_ctx.m_agg);
            r("C", sys_ctx.m_c); r("D", sys_ctx.m_d); r("S", sys_ctx.m_s);
            Serial.println("FINAL_REPORT_END");
            
            xSemaphoreGive(sem_sporadic); // Force Task S to wake up and terminate
        }
    }
    vTaskDelay(10);
}