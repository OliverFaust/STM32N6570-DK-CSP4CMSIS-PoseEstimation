/* RAM trace ring (CSP4CMSIS csp_trace_ring.h; defined in
 * CSP/csp_trace_board.c), DebugTrace configuration only:
 * a second linker script next to STM32N657xx.ld. INSERT places .csp_trace
 * after ._user_heap_stack in AXISRAM1_2_S, below the main stack at the top
 * of that region. NOLOAD: the debugger's load does not write it;
 * csp_trace_ring_init() initialises it. */
SECTIONS
{
  .csp_trace (NOLOAD) :
  {
    . = ALIGN(32);
    KEEP(*(.csp_trace))
    . = ALIGN(32);
  }
  ASSERT(ADDR(.csp_trace) + SIZEOF(.csp_trace) <= _sstack, "Error: .csp_trace overlaps the main stack")
  ASSERT(_eheap <= ADDR(.csp_trace), "Error: .csp_trace overlaps the newlib heap")
}
INSERT AFTER ._user_heap_stack;
