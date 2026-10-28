/*
 * csp_trace_board.c: the board's part of CSP4CMSIS's RAM ring trace backend
 * (csp/csp_trace_ring.h), DebugTrace configuration: the ring in section
 * .csp_trace (NOLOAD, CSP/csp_trace.ld), made non-cacheable by MPU region
 * CSP_TRACE_MPU_REGION, initialised before every other static constructor,
 * and the heartbeat from the FreeRTOS tick hook. The thread hooks are in
 * Inc/FreeRTOSConfig.h. Compiled to nothing unless CSP4CMSIS_TRACE is defined.
 */
#if defined(CSP4CMSIS_TRACE)

#include "stm32n6xx.h"
#include "FreeRTOS.h"
#include "csp/csp_trace_ring.h"

#define CSP_TRACE_MPU_REGION     0U
#define CSP_TRACE_MPU_ATTR       0U             /* MAIR0 attribute index */

struct csp_trace_ring csp_trace_ring __attribute__((section(".csp_trace"), aligned(32)));

static void mpu_init(void)
{
  uint32_t base = (uint32_t)&csp_trace_ring;
  uint32_t limit = base + sizeof(csp_trace_ring) - 1U;
  uint32_t regions = (MPU->TYPE & MPU_TYPE_DREGION_Msk) >> MPU_TYPE_DREGION_Pos;
  uint32_t i;

  SCB_CleanInvalidateDCache_by_Addr(&csp_trace_ring, sizeof(csp_trace_ring));
  MPU->CTRL = 0U;
  __DSB();
  __ISB();
  for (i = 0U; i < regions; i++) {
    MPU->RNR = i;
    MPU->RLAR = 0U;
  }
  MPU->MAIR0 = ARM_MPU_ATTR(ARM_MPU_ATTR_NON_CACHEABLE, ARM_MPU_ATTR_NON_CACHEABLE) << (8U * CSP_TRACE_MPU_ATTR);
  MPU->MAIR1 = 0U;
  ARM_MPU_SetRegion(CSP_TRACE_MPU_REGION, ARM_MPU_RBAR(base, ARM_MPU_SH_NON, 0U, 1U, 1U),
                    ARM_MPU_RLAR(limit, CSP_TRACE_MPU_ATTR));
  MPU->CTRL = MPU_CTRL_PRIVDEFENA_Msk | MPU_CTRL_ENABLE_Msk;
  __DSB();
  __ISB();
}

/* Before every other static constructor, so before the first event. */
__attribute__((constructor(101))) static void csp_trace_board_init(void)
{
  mpu_init();
  csp_trace_ring_init();
}

void vApplicationTickHook(void)
{
  csp_trace_ring_tick(configTICK_RATE_HZ);
}

#endif /* CSP4CMSIS_TRACE */
