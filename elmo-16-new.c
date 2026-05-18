#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <stdint.h>
/****************************************************************************/
#include "ecrt.h"




#ifndef EC_WC_COMPLETE
#define RTLOG_EC_WC_COMPLETE 2 /* IgH 默认 EC_WC_COMPLETE */
#else
#define RTLOG_EC_WC_COMPLETE EC_WC_COMPLETE
#endif
/****************************************************************************/

#define MAX_SERVO_NUM 16


#define RTLOG_RING_SIZE 256
#define RTLOG_RING_MASK (RTLOG_RING_SIZE - 1)
#define RTLOG_FLUSH_MAX_LINES 32
/* ServoCMD=1 且内部 OP_ENABLED 与 6041 不同步（RESYNC_OP_EN）时追加写入，仅在 1Hz rtlog_flush 中打开/写入 */
#define RTLOG_RESYNC_LOG_FILE "elmo_resync_op_en.log"

enum {
    RTLOG_OP_SM_CHANGE           = 0x01, /* CiA402 内部 ServoState 变化 */
    RTLOG_OP_SW_OP_LOST          = 0x02, /* 状态字 bit2 由 1→0（驱动侧掉使能表象） */
    RTLOG_OP_SW_FAULT_RISE       = 0x03, /* Fault 位 0→1 */
    RTLOG_OP_SW_QSTOP_RISE       = 0x04, /* Quick stop 位 0→1 */
    RTLOG_OP_ALL_ENABLE_LOST     = 0x05, /* 全轴 OP 位检查由通过变失败 */
    RTLOG_OP_SLAVES_OK_LOST      = 0x06, /* AL=OP 条件由满足变不满足（1Hz 采样） */
    RTLOG_OP_DOMAIN_OUT_WC_DROP  = 0x07, /* 输出域 WC 由 complete 变非 complete */
    RTLOG_OP_DOMAIN_IN_WC_DROP   = 0x08, /* 输入域 */
    /* ServoCMD==1 但内部仍停在 OP_ENABLED，6041 已无 OP：原逻辑只发 0x0F，与驱动不同步会卡死 */
    RTLOG_OP_RESYNC_FROM_OP_EN   = 0x09,
};

typedef struct {
    uint64_t cycle;
    uint64_t app_ns;
    uint16_t status_word;
    uint16_t control_word;
    uint16_t param;
    uint8_t axis;
    uint8_t opcode;
    uint8_t sm_old;
    uint8_t sm_new;
} rtlog_evt_t;

static rtlog_evt_t rtlog_ring[RTLOG_RING_SIZE];
static uint32_t rtlog_head;
static uint32_t rtlog_tail_flush;
static uint64_t g_rtlog_cycle;
static uint64_t g_rtlog_app_ns;
static FILE *g_rtlog_resync_fp;

static void rtlog_push(uint8_t axis, uint8_t opcode, uint16_t status_word,
    uint16_t control_word, uint8_t sm_old, uint8_t sm_new, uint16_t param)
{
    uint32_t h = rtlog_head;
    rtlog_evt_t *e = &rtlog_ring[h & RTLOG_RING_MASK];

    e->cycle = g_rtlog_cycle;
    e->app_ns = g_rtlog_app_ns;
    e->axis = axis;
    e->opcode = opcode;
    e->status_word = status_word;
    e->control_word = control_word;
    e->sm_old = sm_old;
    e->sm_new = sm_new;
    e->param = param;
    rtlog_head = h + 1;
    if (rtlog_head - rtlog_tail_flush > RTLOG_RING_SIZE) {
        rtlog_tail_flush = rtlog_head - (RTLOG_RING_SIZE - 1);
    }
}

static const char *rtlog_op_name(uint8_t op)
{
    switch (op) {
    case RTLOG_OP_SM_CHANGE: return "SM_CHANGE";
    case RTLOG_OP_SW_OP_LOST: return "SW_OP_LOST";
    case RTLOG_OP_SW_FAULT_RISE: return "SW_FAULT_RISE";
    case RTLOG_OP_SW_QSTOP_RISE: return "SW_QSTOP_RISE";
    case RTLOG_OP_ALL_ENABLE_LOST: return "ALL_ENABLE_LOST";
    case RTLOG_OP_SLAVES_OK_LOST: return "SLAVES_OK_LOST";
    case RTLOG_OP_DOMAIN_OUT_WC_DROP: return "DOM_OUT_WC";
    case RTLOG_OP_DOMAIN_IN_WC_DROP: return "DOM_IN_WC";
    case RTLOG_OP_RESYNC_FROM_OP_EN: return "RESYNC_OP_EN";
    default: return "?";
    }
}

static void rtlog_flush(void)
{
    uint32_t h = rtlog_head;
    unsigned n = 0;

    while (rtlog_tail_flush != h && n < RTLOG_FLUSH_MAX_LINES) {
        rtlog_evt_t *e = &rtlog_ring[rtlog_tail_flush & RTLOG_RING_MASK];

        printf("[RTLOG] cyc=%llu app_ns=%llu ax=%u %s(0x%02x) st=0x%04x cw=0x%04x sm=%u->%u par=0x%x\n",
            (unsigned long long) e->cycle,
            (unsigned long long) e->app_ns,
            (unsigned) e->axis,
            rtlog_op_name(e->opcode),
            (unsigned) e->opcode,
            (unsigned) e->status_word,
            (unsigned) e->control_word,
            (unsigned) e->sm_old,
            (unsigned) e->sm_new,
            (unsigned) e->param);
        if (e->opcode == RTLOG_OP_RESYNC_FROM_OP_EN) {
            if (!g_rtlog_resync_fp) {
                g_rtlog_resync_fp = fopen(RTLOG_RESYNC_LOG_FILE, "a");
                if (!g_rtlog_resync_fp) {
                    fprintf(stderr, "[RTLOG] fopen %s failed: %s\n",
                        RTLOG_RESYNC_LOG_FILE, strerror(errno));
                }
            }
            if (g_rtlog_resync_fp) {
                fprintf(g_rtlog_resync_fp,
                    "[RTLOG] cyc=%llu app_ns=%llu ax=%u %s(0x%02x) st=0x%04x cw=0x%04x sm=%u->%u par=0x%x\n",
                    (unsigned long long) e->cycle,
                    (unsigned long long) e->app_ns,
                    (unsigned) e->axis,
                    rtlog_op_name(e->opcode),
                    (unsigned) e->opcode,
                    (unsigned) e->status_word,
                    (unsigned) e->control_word,
                    (unsigned) e->sm_old,
                    (unsigned) e->sm_new,
                    (unsigned) e->param);
                fflush(g_rtlog_resync_fp);
            }
        }
        rtlog_tail_flush++;
        n++;
    }
}

// Application parameters
#define FREQUENCY 1000

#define CLOCK_TO_USE CLOCK_REALTIME
#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)

#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

/* DS402 device control (state machine) states */
enum MC_T_CIA402_STATE
{
    DRV_DEV_STATE_NOT_READY         = 0,          /* Not ready to switch on : Status Word x0xx 0000 */
    DRV_DEV_STATE_SWITCHON_DIS      = 1,          /* Switch on disabled     : Status Word x1xx 0000 */
    DRV_DEV_STATE_READY_TO_SWITCHON = 2,          /* Ready to switch on     : Status Word x01x 0001 */
    DRV_DEV_STATE_SWITCHED_ON       = 3,          /* Switched on            : Status Word x011 0011 */
    DRV_DEV_STATE_OP_ENABLED        = 4,          /* Operation enabled      : Status Word x011 0111 */
    DRV_DEV_STATE_QUICK_STOP        = 5,          /* Quick stop active      : Status Word 0000 0111 */
    DRV_DEV_STATE_MALFCT_REACTION   = 6,          /* Malfunction/Fault reaction active Status Word (xxxx 1111) oder (xx10 1111) */
    DRV_DEV_STATE_MALFUNCTION       = 7           /* Malfunction/Fault                 */
};

/* DS402 object 0x6041: Status word */
#define DRV_STAT_MASK          	    	0x004F          /* Status mask */
#define DRV_STAT_NONE          	    	0x0000          /* Status none */
#define DRV_STAT_RDY_SWITCH_ON          0x0001          /* Bit 0: Ready to switch on */
#define DRV_STAT_SWITCHED_ON            0x0002          /* Bit 1: Switched On */
#define DRV_STAT_OP_ENABLED             0x0004          /* Bit 2: Operation enabled */
#define DRV_STAT_FAULT                  0x0008          /* Bit 3: Fault */
#define DRV_STAT_VOLTAGE_ENABLED        0x0010          /* Bit 4: Optional bit: Voltage enabled */
#define DRV_STAT_QUICK_STOP             0x0020          /* Bit 5: Optional bit: Quick stop      */
#define DRV_STAT_SWITCH_ON_DIS          0x0040          /* Bit 6: Switch on disabled */
#define DRV_STAT_WARNING                0x0080          /* Bit 8: Warning */
#define DRV_STAT_STATUS_TOGGLE          0x0400          /* Bit 10: Optional bit: Status toggle (csp, csv mode) */
#define DRV_STAT_VELOCITY_ZERO          0x0400          /* Bit 10: Optional bit: Velocity 0 (ip mode) */
#define DRV_STAT_OP_MODE_CSP            0x1000          /* Bit 12: Optional bit: CSP drive follows the command value */
#define DRV_STAT_FOLLOW_ERR             0x2000          /* Bit 13: Optional bit: Following error (csp, csv mode) */
#define DRV_STAT_RUNNING                0x4000          /* Bit 14: Running */
#define DRV_STAT_IDLE                   0x8000          /* Bit 15: Idle */

/* DS402 drive/device control commands */
#define DRV_CTRL_CMD_MASK               0x008F          /* Control commands Mask */
#define DRV_CTRL_CMD_SHUTDOWN           0x06          /* Shutdown (Transition 2, 6, 8) */
#define DRV_CTRL_CMD_SWITCHON           0x07          /* Switch On (Transition 3) */
#define DRV_CTRL_CMD_DIS_VOLTAGE        0x00          /* Disable Voltage (Transition 7, 9, 10, 12) */
#define DRV_CTRL_CMD_DIS_VOLTAGE_MASK   0x82          /* Disable Voltage Mask */
#define DRV_CTRL_CMD_QUICK_STOP         0x02          /* Quick Stop (Transition 7, 10, 11) */
#define DRV_CTRL_CMD_QUICK_STOP_MASK    0x86          /* Disable Voltage Mask */
#define DRV_CTRL_CMD_DIS_OPERATION      0x07          /* Disable Operation (Transition 5) */
#define DRV_CTRL_CMD_ENA_OPERATION      0x0F          /* Enable Operation (Transition 4) */
#define DRV_CRTL_FAULT_RESET            0x0080          /* Reset Malfunction (0->1 edge ) (Transition 15) */

#define DRV_STAT_REON                   0x0001       	/*该参数主要用于使能一次不成功而程序自动再使能*/

static uint32_t My_Servo_Slave_State;
static int32_t ServoCMD;
static int32_t bClearFault;

/****************************************************************************/
// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domainInput = NULL;
static ec_domain_state_t domainInput_state = {};

static ec_domain_t *domainOutput = NULL;
static ec_domain_state_t domainOutput_state = {};

static ec_slave_config_t *sc_motor[MAX_SERVO_NUM];
static ec_slave_config_state_t sc_motor_state[MAX_SERVO_NUM] = {};

/****************************************************************************/
// process data
static uint8_t *domainOutput_pd = NULL;
static uint8_t *domainInput_pd = NULL;

#define MOTOR_SERVO 0, 0
#define MOTOR_SERV1 0, 1
#define MOTOR_SERV2 0, 2
#define MOTOR_SERV3 0, 3
#define MOTOR_SERV4 0, 4
#define MOTOR_SERV5 0, 5
#define MOTOR_SERV6 0, 6
#define MOTOR_SERV7 0, 7
#define MOTOR_SERV8 0, 8
#define MOTOR_SERV9 0, 9
#define MOTOR_SERV10 0, 10
#define MOTOR_SERV11 0, 11
#define MOTOR_SERV12 0, 12
#define MOTOR_SERV13 0, 13
#define MOTOR_SERV14 0, 14
#define MOTOR_SERV15 0, 15

#define MotorSlavePos 0x0000009A, 0x00030924

// offsets for PDO entries
static uint32_t motor_cntlwd[MAX_SERVO_NUM];
static uint32_t motor_tarvel[MAX_SERVO_NUM];
static uint32_t motor_tarpos[MAX_SERVO_NUM];
static uint32_t motor_tartor[MAX_SERVO_NUM];
static uint32_t motor_tarmdop[MAX_SERVO_NUM];
static uint32_t motor_status[MAX_SERVO_NUM];
static uint32_t motor_actpos[MAX_SERVO_NUM];
static uint32_t motor_acttor[MAX_SERVO_NUM];
static uint32_t motor_actvel[MAX_SERVO_NUM];
static uint32_t motor_actmdop[MAX_SERVO_NUM];

const static ec_pdo_entry_reg_t domainOutput_regs[] = {
    { MOTOR_SERVO, MotorSlavePos, 0x6040, 0, &motor_cntlwd[0], NULL },
    { MOTOR_SERVO, MotorSlavePos, 0x6060, 0, &motor_tarmdop[0], NULL },
    { MOTOR_SERVO, MotorSlavePos, 0x6071, 0, &motor_tartor[0], NULL },
    { MOTOR_SERVO, MotorSlavePos, 0x607a, 0, &motor_tarpos[0], NULL },
    { MOTOR_SERVO, MotorSlavePos, 0x60ff, 0, &motor_tarvel[0], NULL },

    { MOTOR_SERV1, MotorSlavePos, 0x6040, 0, &motor_cntlwd[1], NULL },
    { MOTOR_SERV1, MotorSlavePos, 0x6060, 0, &motor_tarmdop[1], NULL },
    { MOTOR_SERV1, MotorSlavePos, 0x6071, 0, &motor_tartor[1], NULL },
    { MOTOR_SERV1, MotorSlavePos, 0x607a, 0, &motor_tarpos[1], NULL },
    { MOTOR_SERV1, MotorSlavePos, 0x60ff, 0, &motor_tarvel[1], NULL },

    { MOTOR_SERV2, MotorSlavePos, 0x6040, 0, &motor_cntlwd[2], NULL },
    { MOTOR_SERV2, MotorSlavePos, 0x6060, 0, &motor_tarmdop[2], NULL },
    { MOTOR_SERV2, MotorSlavePos, 0x6071, 0, &motor_tartor[2], NULL },
    { MOTOR_SERV2, MotorSlavePos, 0x607a, 0, &motor_tarpos[2], NULL },
    { MOTOR_SERV2, MotorSlavePos, 0x60ff, 0, &motor_tarvel[2], NULL },

    { MOTOR_SERV3, MotorSlavePos, 0x6040, 0, &motor_cntlwd[3], NULL },
    { MOTOR_SERV3, MotorSlavePos, 0x6060, 0, &motor_tarmdop[3], NULL },
    { MOTOR_SERV3, MotorSlavePos, 0x6071, 0, &motor_tartor[3], NULL },
    { MOTOR_SERV3, MotorSlavePos, 0x607a, 0, &motor_tarpos[3], NULL },
    { MOTOR_SERV3, MotorSlavePos, 0x60ff, 0, &motor_tarvel[3], NULL },

    { MOTOR_SERV4, MotorSlavePos, 0x6040, 0, &motor_cntlwd[4], NULL },
    { MOTOR_SERV4, MotorSlavePos, 0x6060, 0, &motor_tarmdop[4], NULL },
    { MOTOR_SERV4, MotorSlavePos, 0x6071, 0, &motor_tartor[4], NULL },
    { MOTOR_SERV4, MotorSlavePos, 0x607a, 0, &motor_tarpos[4], NULL },
    { MOTOR_SERV4, MotorSlavePos, 0x60ff, 0, &motor_tarvel[4], NULL },

    { MOTOR_SERV5, MotorSlavePos, 0x6040, 0, &motor_cntlwd[5], NULL },
    { MOTOR_SERV5, MotorSlavePos, 0x6060, 0, &motor_tarmdop[5], NULL },
    { MOTOR_SERV5, MotorSlavePos, 0x6071, 0, &motor_tartor[5], NULL },
    { MOTOR_SERV5, MotorSlavePos, 0x607a, 0, &motor_tarpos[5], NULL },
    { MOTOR_SERV5, MotorSlavePos, 0x60ff, 0, &motor_tarvel[5], NULL },

    { MOTOR_SERV6, MotorSlavePos, 0x6040, 0, &motor_cntlwd[6], NULL },
    { MOTOR_SERV6, MotorSlavePos, 0x6060, 0, &motor_tarmdop[6], NULL },
    { MOTOR_SERV6, MotorSlavePos, 0x6071, 0, &motor_tartor[6], NULL },
    { MOTOR_SERV6, MotorSlavePos, 0x607a, 0, &motor_tarpos[6], NULL },
    { MOTOR_SERV6, MotorSlavePos, 0x60ff, 0, &motor_tarvel[6], NULL },

    { MOTOR_SERV7, MotorSlavePos, 0x6040, 0, &motor_cntlwd[7], NULL },
    { MOTOR_SERV7, MotorSlavePos, 0x6060, 0, &motor_tarmdop[7], NULL },
    { MOTOR_SERV7, MotorSlavePos, 0x6071, 0, &motor_tartor[7], NULL },
    { MOTOR_SERV7, MotorSlavePos, 0x607a, 0, &motor_tarpos[7], NULL },
    { MOTOR_SERV7, MotorSlavePos, 0x60ff, 0, &motor_tarvel[7], NULL },

    { MOTOR_SERV8, MotorSlavePos, 0x6040, 0, &motor_cntlwd[8], NULL },
    { MOTOR_SERV8, MotorSlavePos, 0x6060, 0, &motor_tarmdop[8], NULL },
    { MOTOR_SERV8, MotorSlavePos, 0x6071, 0, &motor_tartor[8], NULL },
    { MOTOR_SERV8, MotorSlavePos, 0x607a, 0, &motor_tarpos[8], NULL },
    { MOTOR_SERV8, MotorSlavePos, 0x60ff, 0, &motor_tarvel[8], NULL },

    { MOTOR_SERV9, MotorSlavePos, 0x6040, 0, &motor_cntlwd[9], NULL },
    { MOTOR_SERV9, MotorSlavePos, 0x6060, 0, &motor_tarmdop[9], NULL },
    { MOTOR_SERV9, MotorSlavePos, 0x6071, 0, &motor_tartor[9], NULL },
    { MOTOR_SERV9, MotorSlavePos, 0x607a, 0, &motor_tarpos[9], NULL },
    { MOTOR_SERV9, MotorSlavePos, 0x60ff, 0, &motor_tarvel[9], NULL },

    { MOTOR_SERV10, MotorSlavePos, 0x6040, 0, &motor_cntlwd[10], NULL },
    { MOTOR_SERV10, MotorSlavePos, 0x6060, 0, &motor_tarmdop[10], NULL },
    { MOTOR_SERV10, MotorSlavePos, 0x6071, 0, &motor_tartor[10], NULL },
    { MOTOR_SERV10, MotorSlavePos, 0x607a, 0, &motor_tarpos[10], NULL },
    { MOTOR_SERV10, MotorSlavePos, 0x60ff, 0, &motor_tarvel[10], NULL },

    { MOTOR_SERV11, MotorSlavePos, 0x6040, 0, &motor_cntlwd[11], NULL },
    { MOTOR_SERV11, MotorSlavePos, 0x6060, 0, &motor_tarmdop[11], NULL },
    { MOTOR_SERV11, MotorSlavePos, 0x6071, 0, &motor_tartor[11], NULL },
    { MOTOR_SERV11, MotorSlavePos, 0x607a, 0, &motor_tarpos[11], NULL },
    { MOTOR_SERV11, MotorSlavePos, 0x60ff, 0, &motor_tarvel[11], NULL },

    { MOTOR_SERV12, MotorSlavePos, 0x6040, 0, &motor_cntlwd[12], NULL },
    { MOTOR_SERV12, MotorSlavePos, 0x6060, 0, &motor_tarmdop[12], NULL },
    { MOTOR_SERV12, MotorSlavePos, 0x6071, 0, &motor_tartor[12], NULL },
    { MOTOR_SERV12, MotorSlavePos, 0x607a, 0, &motor_tarpos[12], NULL },
    { MOTOR_SERV12, MotorSlavePos, 0x60ff, 0, &motor_tarvel[12], NULL },

    { MOTOR_SERV13, MotorSlavePos, 0x6040, 0, &motor_cntlwd[13], NULL },
    { MOTOR_SERV13, MotorSlavePos, 0x6060, 0, &motor_tarmdop[13], NULL },
    { MOTOR_SERV13, MotorSlavePos, 0x6071, 0, &motor_tartor[13], NULL },
    { MOTOR_SERV13, MotorSlavePos, 0x607a, 0, &motor_tarpos[13], NULL },
    { MOTOR_SERV13, MotorSlavePos, 0x60ff, 0, &motor_tarvel[13], NULL },

    { MOTOR_SERV14, MotorSlavePos, 0x6040, 0, &motor_cntlwd[14], NULL },
    { MOTOR_SERV14, MotorSlavePos, 0x6060, 0, &motor_tarmdop[14], NULL },
    { MOTOR_SERV14, MotorSlavePos, 0x6071, 0, &motor_tartor[14], NULL },
    { MOTOR_SERV14, MotorSlavePos, 0x607a, 0, &motor_tarpos[14], NULL },
    { MOTOR_SERV14, MotorSlavePos, 0x60ff, 0, &motor_tarvel[14], NULL },

    { MOTOR_SERV15, MotorSlavePos, 0x6040, 0, &motor_cntlwd[15], NULL },
    { MOTOR_SERV15, MotorSlavePos, 0x6060, 0, &motor_tarmdop[15], NULL },
    { MOTOR_SERV15, MotorSlavePos, 0x6071, 0, &motor_tartor[15], NULL },
    { MOTOR_SERV15, MotorSlavePos, 0x607a, 0, &motor_tarpos[15], NULL },
    { MOTOR_SERV15, MotorSlavePos, 0x60ff, 0, &motor_tarvel[15], NULL },
    {}
};

const static ec_pdo_entry_reg_t domainInput_regs[] = {
    { MOTOR_SERVO, MotorSlavePos, 0x6041, 0, &motor_status[0], NULL },
    { MOTOR_SERVO, MotorSlavePos, 0x6061, 0, &motor_actmdop[0], NULL },
    { MOTOR_SERVO, MotorSlavePos, 0x6064, 0, &motor_actpos[0], NULL },
    { MOTOR_SERVO, MotorSlavePos, 0x606c, 0, &motor_actvel[0], NULL },
    { MOTOR_SERVO, MotorSlavePos, 0x6078, 0, &motor_acttor[0], NULL },

    { MOTOR_SERV1, MotorSlavePos, 0x6041, 0, &motor_status[1], NULL },
    { MOTOR_SERV1, MotorSlavePos, 0x6061, 0, &motor_actmdop[1], NULL },
    { MOTOR_SERV1, MotorSlavePos, 0x6064, 0, &motor_actpos[1], NULL },
    { MOTOR_SERV1, MotorSlavePos, 0x606c, 0, &motor_actvel[1], NULL },
    { MOTOR_SERV1, MotorSlavePos, 0x6078, 0, &motor_acttor[1], NULL },

    { MOTOR_SERV2, MotorSlavePos, 0x6041, 0, &motor_status[2], NULL },
    { MOTOR_SERV2, MotorSlavePos, 0x6061, 0, &motor_actmdop[2], NULL },
    { MOTOR_SERV2, MotorSlavePos, 0x6064, 0, &motor_actpos[2], NULL },
    { MOTOR_SERV2, MotorSlavePos, 0x606c, 0, &motor_actvel[2], NULL },
    { MOTOR_SERV2, MotorSlavePos, 0x6078, 0, &motor_acttor[2], NULL },

    { MOTOR_SERV3, MotorSlavePos, 0x6041, 0, &motor_status[3], NULL },
    { MOTOR_SERV3, MotorSlavePos, 0x6061, 0, &motor_actmdop[3], NULL },
    { MOTOR_SERV3, MotorSlavePos, 0x6064, 0, &motor_actpos[3], NULL },
    { MOTOR_SERV3, MotorSlavePos, 0x606c, 0, &motor_actvel[3], NULL },
    { MOTOR_SERV3, MotorSlavePos, 0x6078, 0, &motor_acttor[3], NULL },

    { MOTOR_SERV4, MotorSlavePos, 0x6041, 0, &motor_status[4], NULL },
    { MOTOR_SERV4, MotorSlavePos, 0x6061, 0, &motor_actmdop[4], NULL },
    { MOTOR_SERV4, MotorSlavePos, 0x6064, 0, &motor_actpos[4], NULL },
    { MOTOR_SERV4, MotorSlavePos, 0x606c, 0, &motor_actvel[4], NULL },
    { MOTOR_SERV4, MotorSlavePos, 0x6078, 0, &motor_acttor[4], NULL },

    { MOTOR_SERV5, MotorSlavePos, 0x6041, 0, &motor_status[5], NULL },
    { MOTOR_SERV5, MotorSlavePos, 0x6061, 0, &motor_actmdop[5], NULL },
    { MOTOR_SERV5, MotorSlavePos, 0x6064, 0, &motor_actpos[5], NULL },
    { MOTOR_SERV5, MotorSlavePos, 0x606c, 0, &motor_actvel[5], NULL },
    { MOTOR_SERV5, MotorSlavePos, 0x6078, 0, &motor_acttor[5], NULL },

    { MOTOR_SERV6, MotorSlavePos, 0x6041, 0, &motor_status[6], NULL },
    { MOTOR_SERV6, MotorSlavePos, 0x6061, 0, &motor_actmdop[6], NULL },
    { MOTOR_SERV6, MotorSlavePos, 0x6064, 0, &motor_actpos[6], NULL },
    { MOTOR_SERV6, MotorSlavePos, 0x606c, 0, &motor_actvel[6], NULL },
    { MOTOR_SERV6, MotorSlavePos, 0x6078, 0, &motor_acttor[6], NULL },

    { MOTOR_SERV7, MotorSlavePos, 0x6041, 0, &motor_status[7], NULL },
    { MOTOR_SERV7, MotorSlavePos, 0x6061, 0, &motor_actmdop[7], NULL },
    { MOTOR_SERV7, MotorSlavePos, 0x6064, 0, &motor_actpos[7], NULL },
    { MOTOR_SERV7, MotorSlavePos, 0x606c, 0, &motor_actvel[7], NULL },
    { MOTOR_SERV7, MotorSlavePos, 0x6078, 0, &motor_acttor[7], NULL },

    { MOTOR_SERV8, MotorSlavePos, 0x6041, 0, &motor_status[8], NULL },
    { MOTOR_SERV8, MotorSlavePos, 0x6061, 0, &motor_actmdop[8], NULL },
    { MOTOR_SERV8, MotorSlavePos, 0x6064, 0, &motor_actpos[8], NULL },
    { MOTOR_SERV8, MotorSlavePos, 0x606c, 0, &motor_actvel[8], NULL },
    { MOTOR_SERV8, MotorSlavePos, 0x6078, 0, &motor_acttor[8], NULL },

    { MOTOR_SERV9, MotorSlavePos, 0x6041, 0, &motor_status[9], NULL },
    { MOTOR_SERV9, MotorSlavePos, 0x6061, 0, &motor_actmdop[9], NULL },
    { MOTOR_SERV9, MotorSlavePos, 0x6064, 0, &motor_actpos[9], NULL },
    { MOTOR_SERV9, MotorSlavePos, 0x606c, 0, &motor_actvel[9], NULL },
    { MOTOR_SERV9, MotorSlavePos, 0x6078, 0, &motor_acttor[9], NULL },

    { MOTOR_SERV10, MotorSlavePos, 0x6041, 0, &motor_status[10], NULL },
    { MOTOR_SERV10, MotorSlavePos, 0x6061, 0, &motor_actmdop[10], NULL },
    { MOTOR_SERV10, MotorSlavePos, 0x6064, 0, &motor_actpos[10], NULL },
    { MOTOR_SERV10, MotorSlavePos, 0x606c, 0, &motor_actvel[10], NULL },
    { MOTOR_SERV10, MotorSlavePos, 0x6078, 0, &motor_acttor[10], NULL },

    { MOTOR_SERV11, MotorSlavePos, 0x6041, 0, &motor_status[11], NULL },
    { MOTOR_SERV11, MotorSlavePos, 0x6061, 0, &motor_actmdop[11], NULL },
    { MOTOR_SERV11, MotorSlavePos, 0x6064, 0, &motor_actpos[11], NULL },
    { MOTOR_SERV11, MotorSlavePos, 0x606c, 0, &motor_actvel[11], NULL },
    { MOTOR_SERV11, MotorSlavePos, 0x6078, 0, &motor_acttor[11], NULL },

    { MOTOR_SERV12, MotorSlavePos, 0x6041, 0, &motor_status[12], NULL },
    { MOTOR_SERV12, MotorSlavePos, 0x6061, 0, &motor_actmdop[12], NULL },
    { MOTOR_SERV12, MotorSlavePos, 0x6064, 0, &motor_actpos[12], NULL },
    { MOTOR_SERV12, MotorSlavePos, 0x606c, 0, &motor_actvel[12], NULL },
    { MOTOR_SERV12, MotorSlavePos, 0x6078, 0, &motor_acttor[12], NULL },

    { MOTOR_SERV13, MotorSlavePos, 0x6041, 0, &motor_status[13], NULL },
    { MOTOR_SERV13, MotorSlavePos, 0x6061, 0, &motor_actmdop[13], NULL },
    { MOTOR_SERV13, MotorSlavePos, 0x6064, 0, &motor_actpos[13], NULL },
    { MOTOR_SERV13, MotorSlavePos, 0x606c, 0, &motor_actvel[13], NULL },
    { MOTOR_SERV13, MotorSlavePos, 0x6078, 0, &motor_acttor[13], NULL },

    { MOTOR_SERV14, MotorSlavePos, 0x6041, 0, &motor_status[14], NULL },
    { MOTOR_SERV14, MotorSlavePos, 0x6061, 0, &motor_actmdop[14], NULL },
    { MOTOR_SERV14, MotorSlavePos, 0x6064, 0, &motor_actpos[14], NULL },
    { MOTOR_SERV14, MotorSlavePos, 0x606c, 0, &motor_actvel[14], NULL },
    { MOTOR_SERV14, MotorSlavePos, 0x6078, 0, &motor_acttor[14], NULL },

    { MOTOR_SERV15, MotorSlavePos, 0x6041, 0, &motor_status[15], NULL },
    { MOTOR_SERV15, MotorSlavePos, 0x6061, 0, &motor_actmdop[15], NULL },
    { MOTOR_SERV15, MotorSlavePos, 0x6064, 0, &motor_actpos[15], NULL },
    { MOTOR_SERV15, MotorSlavePos, 0x606c, 0, &motor_actvel[15], NULL },
    { MOTOR_SERV15, MotorSlavePos, 0x6078, 0, &motor_acttor[15], NULL },
    {}
};

/*****************************************************************************/
static ec_pdo_entry_info_t motor_pdo_entries_output[] = {
    { 0x6040, 0x00, 16 },
    { 0x6060, 0x00, 8 },
    { 0x6071, 0x00, 16 },
    { 0x607a, 0x00, 32 },
    { 0x60ff, 0x00, 32 },
};

static ec_pdo_entry_info_t motor_pdo_entries_input[] = {
    { 0x6041, 0x00, 16 },
    { 0x6061, 0x00, 8 },
    { 0x6064, 0x00, 32 },
    { 0x606c, 0x00, 32 },
    { 0x6078, 0x00, 16 },
};

static ec_pdo_info_t motor_pdo_1600[] = {
    { 0x1607, 5, motor_pdo_entries_output },
};

static ec_pdo_info_t motor_pdo_1a00[] = {
    { 0x1A08, 5, motor_pdo_entries_input},
};

static ec_sync_info_t motor_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, motor_pdo_1600, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, motor_pdo_1a00, EC_WD_DISABLE},
    {0xff}
};

/*****************************************************************************/
static void ecSlavesControlSet(uint32_t s_num, uint16_t control)
{
    EC_WRITE_U16(domainOutput_pd + motor_cntlwd[s_num], control);
}

static void ecSlavesPositionSet(uint32_t s_num, int32_t position)
{
    EC_WRITE_S32(domainOutput_pd + motor_tarpos[s_num], position);
}

static void ecSlavestorqueSet(uint32_t s_num, int16_t torque)
{
    EC_WRITE_S16(domainOutput_pd + motor_tartor[s_num], torque);
}

static void ecSlavesVelocitySet(uint32_t s_num, int32_t velocity)
{
    EC_WRITE_S32(domainOutput_pd + motor_tarvel[s_num], velocity);
}

static void ecSlavesModeSet(uint32_t s_num, uint8_t mode)
{
    EC_WRITE_U8(domainOutput_pd + motor_tarmdop[s_num], mode);
}

static uint16_t ecSlavesStatusGet(uint32_t s_num)
{
    return EC_READ_U16(domainInput_pd + motor_status[s_num]);
}

static int32_t ecSlavesPositionGet(uint32_t s_num)
{
    return EC_READ_S32(domainInput_pd + motor_actpos[s_num]);
}

static int32_t ecSlavesVelocityGet(uint32_t s_num)
{
    return EC_READ_S32(domainInput_pd + motor_actvel[s_num]);
}

static int16_t ecSlavesTorqueGet(uint32_t s_num)
{
    return EC_READ_S16(domainInput_pd + motor_acttor[s_num]);
}

static uint8_t ecSlavesModeGet(uint32_t s_num)
{
    return EC_READ_U8(domainInput_pd + motor_actmdop[s_num]);
}

/*****************************************************************************/

/*****************************************************************************/

static void check_domain_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domainOutput, &ds);

    if (ds.working_counter != domainOutput_state.working_counter) {
        printf("domainOutput: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domainOutput_state.wc_state) {
        printf("domainOutput: State %u.\n", ds.wc_state);
    }

    domainOutput_state = ds;

    ecrt_domain_state(domainInput, &ds);

    if (ds.working_counter != domainInput_state.working_counter) {
        printf("domainInput: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domainInput_state.wc_state) {
        printf("domainInput: State %u.\n", ds.wc_state);
    }

    domainInput_state = ds;
}

/*****************************************************************************/

static void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding) {
        printf("%u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != master_state.al_states) {
        printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != master_state.link_up) {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    master_state = ms;
}

/*****************************************************************************/

static void check_slave_config_states(void)
{
    ec_slave_config_state_t s[MAX_SERVO_NUM];
    uint32_t i;

    for (i = 0; i < MAX_SERVO_NUM; i++) {
        ecrt_slave_config_state(sc_motor[i], &s[i]);
        sc_motor_state[i] = s[i];
    }
}

static uint32_t check_master_slaves_states(void) {
    uint32_t i;

    if (master_state.al_states != 8) {
        printf("master:%d\n", master_state.al_states);
        return 0;
    }

    for (i = 0; i < MAX_SERVO_NUM; i++) {
        if (sc_motor_state[i].al_state != 8) {
            printf("num:%d, s_state:%d\n", i, sc_motor_state[i].al_state);
            return 0;
        }
        //printf("num:%d, s_state:%d\n", i, sc_motor_state[i].al_state);
    }

    return 1;
}

static int Rb_GetServoOnState(int i_ServoNum)
{
    uint32_t i = 0;
    int32_t ServoEnbleState[MAX_SERVO_NUM] = {0};
    uint16_t usStates[MAX_SERVO_NUM]={0};

    for (i = 0; i < i_ServoNum; i++) {
        usStates[i] = ecSlavesStatusGet(i);
    }

    for (i = 0; i < i_ServoNum; i++) {
        ServoEnbleState[i] = ((usStates[i]) & DRV_STAT_OP_ENABLED);
        if (ServoEnbleState[i] != DRV_STAT_OP_ENABLED) {
            return 0;
        }
    }
    return 1;
}

static void Process_Commands()
{
    static uint16_t ServoStatus[MAX_SERVO_NUM] = {0};
    static uint16_t ServoState[MAX_SERVO_NUM] = {0};
    static uint16_t wControlWord[MAX_SERVO_NUM];
    static uint8_t fault_reset_seq[MAX_SERVO_NUM];
    static uint16_t prev_status[MAX_SERVO_NUM];
    uint32_t MotorActPulse[MAX_SERVO_NUM];

    for (int mIndex = 0; mIndex < MAX_SERVO_NUM; mIndex++) {
        uint8_t sm_before = (uint8_t) ServoState[mIndex];
        uint16_t sw_prev = prev_status[mIndex];

        ServoStatus[mIndex] = ecSlavesStatusGet(mIndex);
        if (ServoState[mIndex] != DRV_DEV_STATE_MALFCT_REACTION) {
            fault_reset_seq[mIndex] = 0;
        }
        switch (ServoState[mIndex]) {
        case DRV_DEV_STATE_NOT_READY:
            //printf("not ready=%x\n", ServoStatus[mIndex]);
            if ((ServoStatus[mIndex] & DRV_STAT_MASK) == DRV_STAT_NONE
                || (ServoStatus[mIndex] & DRV_STAT_MASK) == DRV_STAT_SWITCH_ON_DIS
                || (ServoStatus[mIndex] & DRV_STAT_MASK) == (DRV_STAT_SWITCH_ON_DIS | DRV_STAT_QUICK_STOP)) {
                wControlWord[mIndex] = DRV_CTRL_CMD_SHUTDOWN;
                ServoState[mIndex] = DRV_DEV_STATE_SWITCHON_DIS;
            } else if (ServoStatus[mIndex] & DRV_STAT_FAULT) {
                ServoState[mIndex] = DRV_DEV_STATE_MALFCT_REACTION;
            } else if ((ServoStatus[mIndex] & DRV_STAT_RDY_SWITCH_ON) == DRV_STAT_RDY_SWITCH_ON) {
                ServoState[mIndex] = DRV_DEV_STATE_SWITCHON_DIS;
            }
            break;
        case DRV_DEV_STATE_SWITCHON_DIS:
            //printf("switchon dis=%x\n",ServoStatus[mIndex]);
            if ((ServoStatus[mIndex] & DRV_STAT_MASK) == DRV_STAT_SWITCH_ON_DIS) {
                wControlWord[mIndex] = DRV_CTRL_CMD_SHUTDOWN;
                ServoState[mIndex] = DRV_DEV_STATE_READY_TO_SWITCHON;
            } else if ((ServoStatus[mIndex] & DRV_STAT_MASK) == DRV_STAT_RDY_SWITCH_ON) {
                ServoState[mIndex] = DRV_DEV_STATE_READY_TO_SWITCHON;
            } else if ((ServoStatus[mIndex] & 0x6F) == (DRV_STAT_QUICK_STOP | DRV_STAT_SWITCHED_ON | DRV_STAT_RDY_SWITCH_ON)) {
                ServoState[mIndex] = DRV_DEV_STATE_SWITCHED_ON;
            } else if (ServoStatus[mIndex] & DRV_STAT_FAULT) {
                ServoState[mIndex] = DRV_DEV_STATE_MALFCT_REACTION;
            }
            break;
        case DRV_DEV_STATE_READY_TO_SWITCHON:
            //printf("ready to switch=%x\n",ServoStatus[mIndex]);
            MotorActPulse[mIndex] = ecSlavesPositionGet(mIndex);
            if (MotorActPulse[mIndex] != 0) {
                ecSlavesPositionSet(mIndex, MotorActPulse[mIndex]);
            }
            ecSlavesVelocitySet(mIndex, 0);

            if ((ServoStatus[mIndex] & 0x6F) == (DRV_STAT_QUICK_STOP | DRV_STAT_RDY_SWITCH_ON)) {
                wControlWord[mIndex] = DRV_CTRL_CMD_SWITCHON;
                ServoState[mIndex] = DRV_DEV_STATE_SWITCHED_ON;
            } else if (ServoStatus[mIndex] & DRV_STAT_FAULT) {
                ServoState[mIndex] = DRV_DEV_STATE_MALFCT_REACTION;
            }
            break;
        case DRV_DEV_STATE_SWITCHED_ON:
            //printf("switched on=%x\n",ServoStatus[mIndex]);
            MotorActPulse[mIndex] = ecSlavesPositionGet(mIndex);
            if (MotorActPulse[mIndex] != 0) {
                ecSlavesPositionSet(mIndex, MotorActPulse[mIndex]);
            }
            ecSlavesVelocitySet(mIndex, 0);

            if (ServoCMD == 1) {
                if ((ServoStatus[mIndex] & 0x6F) == (DRV_STAT_QUICK_STOP | DRV_STAT_SWITCHED_ON | DRV_STAT_RDY_SWITCH_ON)) {
                    wControlWord[mIndex] = DRV_CTRL_CMD_ENA_OPERATION;
                    ServoState[mIndex] = DRV_DEV_STATE_OP_ENABLED;
                }
            }
            if (ServoStatus[mIndex] & DRV_STAT_FAULT) {
                ServoState[mIndex] = DRV_DEV_STATE_MALFCT_REACTION;
            }
            break;
        case DRV_DEV_STATE_OP_ENABLED:
            //printf("op enabled=%x\n",ServoStatus[mIndex]);
            if (ServoStatus[mIndex] & DRV_STAT_FAULT) {
                ServoState[mIndex] = DRV_DEV_STATE_MALFCT_REACTION;
            } else if (ServoCMD == 0) {
                MotorActPulse[mIndex] = ecSlavesPositionGet(mIndex);
                if (MotorActPulse[mIndex] != 0) {
                    ecSlavesPositionSet(mIndex, MotorActPulse[mIndex]);
                }
                ecSlavesVelocitySet(mIndex, 0);
                wControlWord[mIndex] = DRV_CTRL_CMD_SHUTDOWN;
                ServoState[mIndex] = DRV_DEV_STATE_SWITCHON_DIS;
            } else if (ServoCMD == 1
                && !(ServoStatus[mIndex] & DRV_STAT_OP_ENABLED)) {
                /* 掉使能后仍认为 OP_ENABLED 且 ServoCMD 一直为 1：只发 0x0F 无法与真实 CiA 状态对齐 */
                uint16_t sw = ServoStatus[mIndex];
                uint16_t st6 = sw & 0x6F;
                uint16_t cw_rd = EC_READ_U16(domainOutput_pd + motor_cntlwd[mIndex]);

                if (g_rtlog_cycle > 5) {
                    rtlog_push((uint8_t) mIndex, RTLOG_OP_RESYNC_FROM_OP_EN, sw, cw_rd,
                        (uint8_t) DRV_DEV_STATE_OP_ENABLED, 0, st6);
                }
                if (st6 == 0x0F) {
                    ServoState[mIndex] = DRV_DEV_STATE_MALFCT_REACTION;
                    wControlWord[mIndex] = DRV_CTRL_CMD_DIS_VOLTAGE;
                } else if (st6 == 0x08) {
                    ServoState[mIndex] = DRV_DEV_STATE_MALFCT_REACTION;
                    wControlWord[mIndex] = DRV_CTRL_CMD_DIS_VOLTAGE;
                } else if (st6 == 0x40
                    || (sw & DRV_STAT_MASK) == DRV_STAT_SWITCH_ON_DIS) {
                    ServoState[mIndex] = DRV_DEV_STATE_SWITCHON_DIS;
                    wControlWord[mIndex] = DRV_CTRL_CMD_SHUTDOWN;
                } else if (st6 == (DRV_STAT_QUICK_STOP | DRV_STAT_SWITCHED_ON
                    | DRV_STAT_RDY_SWITCH_ON)) {
                    ServoState[mIndex] = DRV_DEV_STATE_SWITCHED_ON;
                    wControlWord[mIndex] = DRV_CTRL_CMD_ENA_OPERATION;
                } else if (st6 == (DRV_STAT_QUICK_STOP | DRV_STAT_RDY_SWITCH_ON)) {
                    ServoState[mIndex] = DRV_DEV_STATE_READY_TO_SWITCHON;
                    wControlWord[mIndex] = DRV_CTRL_CMD_SWITCHON;
                } else {
                    ServoState[mIndex] = DRV_DEV_STATE_SWITCHON_DIS;
                    wControlWord[mIndex] = DRV_CTRL_CMD_SHUTDOWN;
                }
            }
            break;
        case DRV_DEV_STATE_MALFCT_REACTION: {
            uint16_t sw = ServoStatus[mIndex];
            uint16_t st6 = sw & 0x6F;

            if (st6 == 0x0F) {
                wControlWord[mIndex] = DRV_CTRL_CMD_DIS_VOLTAGE;
                break;
            }
            if (st6 == 0x40 || (sw & DRV_STAT_MASK) == DRV_STAT_SWITCH_ON_DIS) {
                fault_reset_seq[mIndex] = 0;
                wControlWord[mIndex] = DRV_CTRL_CMD_SHUTDOWN;
                ServoState[mIndex] = DRV_DEV_STATE_SWITCHON_DIS;
                break;
            }
            if (st6 != 0x08) {
                if (!(sw & DRV_STAT_FAULT)) {
                    fault_reset_seq[mIndex] = 0;
                    ServoState[mIndex] = DRV_DEV_STATE_SWITCHON_DIS;
                }
                break;
            }
            if (!bClearFault) {
                wControlWord[mIndex] = DRV_CTRL_CMD_DIS_VOLTAGE;
                fault_reset_seq[mIndex] = 0;
                break;
            }
            if (fault_reset_seq[mIndex] == 0) {
                wControlWord[mIndex] = DRV_CTRL_CMD_DIS_VOLTAGE;
                fault_reset_seq[mIndex] = 1;
            } else if (fault_reset_seq[mIndex] == 1) {
                wControlWord[mIndex] = DRV_CRTL_FAULT_RESET;
                fault_reset_seq[mIndex] = 2;
            } else {
                wControlWord[mIndex] = DRV_CRTL_FAULT_RESET;
            }
            break;
        }
        case DRV_DEV_STATE_MALFUNCTION:
            ServoState[mIndex] = DRV_DEV_STATE_MALFCT_REACTION;
            break;
        default:
            break;
        }

        {
            uint16_t sw = ServoStatus[mIndex];
            uint16_t cw_rd = EC_READ_U16(domainOutput_pd + motor_cntlwd[mIndex]);
            uint8_t sm_after = (uint8_t) ServoState[mIndex];

            if (g_rtlog_cycle > 5) {
                if (sm_after != sm_before) {
                    rtlog_push((uint8_t) mIndex, RTLOG_OP_SM_CHANGE, sw, cw_rd,
                        sm_before, sm_after, 0);
                }
                if ((sw_prev & DRV_STAT_OP_ENABLED) && !(sw & DRV_STAT_OP_ENABLED)) {
                    rtlog_push((uint8_t) mIndex, RTLOG_OP_SW_OP_LOST, sw, cw_rd,
                        sm_before, sm_after, (uint16_t) (sw ^ sw_prev));
                }
                if (!(sw_prev & DRV_STAT_FAULT) && (sw & DRV_STAT_FAULT)) {
                    rtlog_push((uint8_t) mIndex, RTLOG_OP_SW_FAULT_RISE, sw, cw_rd,
                        sm_before, sm_after, 0);
                }
                if (!(sw_prev & DRV_STAT_QUICK_STOP) && (sw & DRV_STAT_QUICK_STOP)) {
                    rtlog_push((uint8_t) mIndex, RTLOG_OP_SW_QSTOP_RISE, sw, cw_rd,
                        sm_before, sm_after, 0);
                }
            }
            prev_status[mIndex] = sw;
        }

        //printf("wControlWord: %x\n", wControlWord[mIndex]);
        ecSlavesControlSet(mIndex, wControlWord[mIndex]);
    }
}

static void myWorkpd()
{
    uint32_t mIndex;
    printf("!!!!!!!!Enable successful\n");
//    MotionContrl(PulseCmd);//解析网络信息
    for (mIndex = 0; mIndex < MAX_SERVO_NUM; mIndex++) {
//        printf("index:%d, torque:%d, pos:%d, vel:%d\n", mIndex, ecSlavesTorqueGet(mIndex), ecSlavesPositionGet(mIndex), ecSlavesVelocityGet(mIndex));
        if (ecSlavesModeGet(mIndex) != 0x9) {
            ecSlavesModeSet(mIndex, 0x9);
        }
//        ecSlavesVelocitySet(mIndex, 19200);   //控制电机
    }
//    printf("      \n");
}

/****************************************************************************/

static void cyclic_task()
{
    struct timespec time;
    int slaves_ok = 0;
    int bServoEnableState;
    static uint32_t counter;
    
    
    static int rtlog_slaves_ok_prev;
    static int rtlog_all_en_prev;
    static unsigned int rtlog_dom_out_wc_prev = 0xFFFFFFFFu;
    static unsigned int rtlog_dom_in_wc_prev = 0xFFFFFFFFu;

    g_rtlog_cycle++;

 // receive process data
    ecrt_master_receive(master);
    ecrt_domain_process(domainOutput);
    ecrt_domain_process(domainInput);

 // check process data state (optional)
    check_domain_state();

    clock_gettime(CLOCK_TO_USE, &time);
    g_rtlog_app_ns = TIMESPEC2NS(time);

    /* IgH: EC_WC_COMPLETE */
    if (rtlog_dom_out_wc_prev != 0xFFFFFFFFu
        && rtlog_dom_out_wc_prev == RTLOG_EC_WC_COMPLETE && domainOutput_state.wc_state != RTLOG_EC_WC_COMPLETE) {
        rtlog_push(0xFF, RTLOG_OP_DOMAIN_OUT_WC_DROP, 0, 0, 0, 0,
            (uint16_t) domainOutput_state.working_counter);
    }
    if (rtlog_dom_in_wc_prev != 0xFFFFFFFFu
        && rtlog_dom_in_wc_prev == RTLOG_EC_WC_COMPLETE && domainInput_state.wc_state != RTLOG_EC_WC_COMPLETE) {
        rtlog_push(0xFF, RTLOG_OP_DOMAIN_IN_WC_DROP, 0, 0, 0, 0,
            (uint16_t) domainInput_state.working_counter);
    }
    rtlog_dom_out_wc_prev = domainOutput_state.wc_state;
    rtlog_dom_in_wc_prev = domainInput_state.wc_state;

    if (counter) {
        counter--;
    } else { // do this at 1 Hz
        counter = 1000;

// check for master state (optional)
        check_master_state();

// check for islave configuration state(s) (optional)
        check_slave_config_states();
        slaves_ok = check_master_slaves_states();
        if (rtlog_slaves_ok_prev == 1 && slaves_ok == 0) {
            rtlog_push(0xFF, RTLOG_OP_SLAVES_OK_LOST, 0, 0, 0, 0,
                (uint16_t) master_state.al_states);
        }
        rtlog_slaves_ok_prev = slaves_ok;
        rtlog_flush();
    }

	ecrt_master_application_time(master, g_rtlog_app_ns);

    ecrt_master_sync_reference_clock(master);

    if (slaves_ok == 1) {
        Process_Commands(); //上使能
        bServoEnableState = Rb_GetServoOnState(MAX_SERVO_NUM);
        if (rtlog_all_en_prev == 1 && !bServoEnableState) {
            uint32_t j;

            for (j = 0; j < MAX_SERVO_NUM; j++) {
                uint16_t ss = ecSlavesStatusGet(j);

                if ((ss & DRV_STAT_OP_ENABLED) != DRV_STAT_OP_ENABLED) {
                    uint16_t cw = EC_READ_U16(domainOutput_pd + motor_cntlwd[j]);

                    rtlog_push((uint8_t) j, RTLOG_OP_ALL_ENABLE_LOST, ss, cw, 0, 0, 0);
                    break;
                }
            }
        }
        rtlog_all_en_prev = bServoEnableState;
        if (bServoEnableState) {//判断所有轴使能
            myWorkpd();
        }
    }

    ecrt_master_sync_slave_clocks(master);
    // send process data
    ecrt_domain_queue(domainOutput);
    ecrt_domain_queue(domainInput);
    ecrt_master_send(master);
}

int32_t init_socket() //网络初始化
{
    return 0;
}
/****************************************************************************/
int main(int argc, char **argv)
{
    uint64_t ulCurrentCnt;
    int32_t ret;
    struct timespec sysTime = {0};

    ret = init_socket();
    if (ret) {
        fprintf(stderr, "Failed to init socket.\n");
        return -1;
    }
// Requests an EtherCAT master for realtime operation.
    master = ecrt_request_master(0); // Index of the master to request.
    if (!master) {
        fprintf(stderr, "Failed to request master.\n");
        return -1;
    }

// Creates a new process data domain
    domainOutput = ecrt_master_create_domain(master);
    if (!domainOutput) {
        fprintf(stderr, "Failed to create output domain.\n");
        return -1;
    }
    domainInput = ecrt_master_create_domain(master);
    if (!domainInput) {
        fprintf(stderr, "Failed to create input domain.\n");
        return -1;
    }

    for (int i = 0; i < MAX_SERVO_NUM; i++) {
        if (!(sc_motor[i] = ecrt_master_slave_config(master, 0, i, MotorSlavePos))) {
            fprintf(stderr, "Failed to get slave configuration %d.\n", i);
            return -1;
        }

        if (ecrt_slave_config_pdos(sc_motor[i], EC_END, motor_syncs)) {
            fprintf(stderr, "Failed to configure PDOs %d.\n", i);
            return -1;
        }
        printf("Configuring PDOs...\n");
    }
    if (ecrt_domain_reg_pdo_entry_list(domainOutput, domainOutput_regs)) {
        fprintf(stderr, "Output PDO entry registration failed!\n");
        return -1;
    }

    if (ecrt_domain_reg_pdo_entry_list(domainInput, domainInput_regs)) {
        fprintf(stderr, "Input PDO entry registration failed!\n");
        return -1;
    }
    for (int i = 0; i < MAX_SERVO_NUM; i++) {
        ecrt_slave_config_dc(sc_motor[i], 0x0300, PERIOD_NS, PERIOD_NS/2, 0, 0);
    }

    usleep(1000);
    printf("Activating master...\n");
    if (ecrt_master_activate(master)) {
        fprintf(stderr, "Failed to activate master!\n");
        return -1;
    }

    if (!(domainOutput_pd = ecrt_domain_data(domainOutput))) {
        fprintf(stderr, "Failed to get output domain phyaddr!\n");
        return -1;
    }

    if (!(domainInput_pd = ecrt_domain_data(domainInput))) {
        fprintf(stderr, "Failed to get input domain phyaddr!\n");
        return -1;
    }

    ServoCMD = 1;
    bClearFault = 1;

    clock_gettime(CLOCK_MONOTONIC, &sysTime);
    ulCurrentCnt = sysTime.tv_sec * 1000000000 + sysTime.tv_nsec;
    while (1) {
        ulCurrentCnt += 1000000;
        sysTime.tv_sec = (ulCurrentCnt) / 1000000000;
        sysTime.tv_nsec = ulCurrentCnt % 1000000000;
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &sysTime, NULL);
        cyclic_task();
    }

    return 0;
}
