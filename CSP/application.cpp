// The CSP4CMSIS process network of the pose estimation application: the
// isp, nn, pp and dp stages of ST's Src/app.c as processes, connected by
// channels (app_application.h declares the stage functions it calls).
#include <cstdio>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "csp/csp4cmsis.h"
#include "app_application.h"

using namespace csp;

struct trigger_t {};

static BufferedChannel<trigger_t, 1, BufferPolicy::KeepNewest> isp_trigger_chan;

extern "C" void csp_signal_isp_vsync(void) {
  isp_trigger_chan.isrWriter().putFromISR(trigger_t{});
}

struct FrameMessage {
  uint32_t buffer_index;
  uint32_t timestamp;
};

struct NNResultMessage {
  uint32_t inf_ms;
  uint32_t period_ms;
};

class ISPWorker : public CSProcessStatic<2048> {
  Chanin<trigger_t> trigger_in;
  Chanout<FrameMessage> out;

 public:
  ISPWorker(Chanin<trigger_t> r, Chanout<FrameMessage> w) : trigger_in(r), out(w) {}

  void run() override {
    trigger_t trig;
    while (true) {
      trigger_in >> trig;
      CAM_IspUpdate();
      FrameMessage msg = { 0, HAL_GetTick() };
      out << msg;
    }
  }
};

class NNWorker : public CSProcessStatic<2048> {
  Chanin<FrameMessage> in;
  Chanout<FrameMessage> out;
  Chanout<FrameMessage> to_npu;
  Chanin<NNResultMessage> from_npu;

 public:
  NNWorker(Chanin<FrameMessage> r, Chanout<FrameMessage> w,
           Chanout<FrameMessage> w_npu, Chanin<NNResultMessage> r_npu)
      : in(r), out(w), to_npu(w_npu), from_npu(r_npu) {}

  void run() override {
    FrameMessage msg;
    NNResultMessage res;
    while (true) {
      in >> msg;
      to_npu << msg;
      from_npu >> res;
      out << msg;
    }
  }
};

class NNWorker_2 : public CSProcessStatic<2048> {
  Chanin<FrameMessage> in;
  Chanout<NNResultMessage> out;
  Chanout<FrameMessage> to_npu;
  Chanin<NNResultMessage> from_npu;

 public:
  NNWorker_2(Chanin<FrameMessage> r, Chanout<NNResultMessage> w,
             Chanout<FrameMessage> w_npu, Chanin<NNResultMessage> r_npu)
      : in(r), out(w), to_npu(w_npu), from_npu(r_npu) {}

  void run() override {
    FrameMessage msg;
    NNResultMessage res;
    while (true) {
      in >> msg;
      to_npu << msg;
      from_npu >> res;
      out << res;
    }
  }
};

class NpuConcentratorProcess : public CSProcessStatic<2048> {
  Chanin<FrameMessage> req_m1_in;
  Chanin<FrameMessage> req_m2_in;
  Chanout<NNResultMessage> resp_m1_out;
  Chanout<NNResultMessage> resp_m2_out;

 public:
  NpuConcentratorProcess(Chanin<FrameMessage> r1, Chanin<FrameMessage> r2,
                         Chanout<NNResultMessage> w1, Chanout<NNResultMessage> w2)
      : req_m1_in(r1), req_m2_in(r2), resp_m1_out(w1), resp_m2_out(w2) {}

  void run() override {
    NN_Hardware_Init();
    uint32_t last_ts_m1 = 0;
    uint32_t last_ts_m2 = 0;
    FrameMessage r1;
    FrameMessage r2;
    Alternative npu_alt(req_m1_in | r1, req_m2_in | r2);
    while (true) {
      int sel = npu_alt.fairSelect();
      if (sel == 0) {
        uint32_t period = r1.timestamp - last_ts_m1;
        last_ts_m1 = r1.timestamp;
        NNResultMessage res = { .inf_ms = 0, .period_ms = period };
        resp_m1_out << res;
      } else {
        uint32_t period = r2.timestamp - last_ts_m2;
        last_ts_m2 = r2.timestamp;
        uint32_t inf_ms = NN_Execute_Inference();
        NNResultMessage res = { .inf_ms = inf_ms, .period_ms = period };
        resp_m2_out << res;
      }
    }
  }
};

class PPWorker : public CSProcessStatic<2048> {
  Chanin<NNResultMessage> in;
  Chanout<display_info_t> out;

 public:
  PPWorker(Chanin<NNResultMessage> r, Chanout<display_info_t> w) : in(r), out(w) {}

  void run() override {
    PP_Hardware_Init();
    NNResultMessage nn_res;
    display_info_t disp_msg;
    while (true) {
      in >> nn_res;
      uint32_t start = HAL_GetTick();
      PP_Execute_Inference(&disp_msg);
      disp_msg.pp_ms = HAL_GetTick() - start;
      disp_msg.inf_ms = nn_res.inf_ms;
      disp_msg.nn_period_ms = nn_res.period_ms;
      out << disp_msg;
    }
  }
};

class DPWorker : public CSProcessStatic<2048> {
  Chanin<display_info_t> in;

 public:
  explicit DPWorker(Chanin<display_info_t> r) : in(r) {}

  void run() override {
    display_info_t msg;
    uint32_t last_stats = HAL_GetTick();
    while (true) {
      in >> msg;
      LCD_Render_Frame(&msg);
      if (HAL_GetTick() - last_stats >= 5000) {
        last_stats = HAL_GetTick();
        printf("stats fps %.2f inf %lums cpu %.1f%%\r\n",
               1000.0 / msg.nn_period_ms, (unsigned long)msg.inf_ms, LCD_Get_Cpu_Load());
      }
    }
  }
};

static constexpr osPriority_t MAIN_APP_PRIORITY = osPriorityBelowNormal;
static constexpr osPriority_t NETWORK_PRIORITY  = osPriorityLow;

alignas(8) static uint32_t mainAppStack[256];
static StaticTask_t mainAppControlBlock;

void MainApp_Task(void* argument) {
  (void)argument;
  osDelay(10);
  printf("\r\n--- Launching STM32N6 AI Parallel Network ---\r\n");

  static Channel<FrameMessage> isp_to_nn_chan;
  static Channel<FrameMessage> isp_to_nn_chan_2;
  static Channel<NNResultMessage> nn_to_pp_chan;
  static BufferedChannel<display_info_t, 1, BufferPolicy::KeepNewest> pp_to_disp_chan;
  static Channel<FrameMessage> nn1_to_npu_chan;
  static Channel<NNResultMessage> npu_to_nn1_chan;
  static Channel<FrameMessage> nn2_to_npu_chan;
  static Channel<NNResultMessage> npu_to_nn2_chan;

  static ISPWorker isp_proc(isp_trigger_chan.reader(), isp_to_nn_chan.writer());
  static NNWorker nn_proc(isp_to_nn_chan.reader(), isp_to_nn_chan_2.writer(),
                          nn1_to_npu_chan.writer(), npu_to_nn1_chan.reader());
  static NNWorker_2 nn_proc_2(isp_to_nn_chan_2.reader(), nn_to_pp_chan.writer(),
                              nn2_to_npu_chan.writer(), npu_to_nn2_chan.reader());
  static NpuConcentratorProcess npu_proc(nn1_to_npu_chan.reader(), nn2_to_npu_chan.reader(),
                                         npu_to_nn1_chan.writer(), npu_to_nn2_chan.writer());
  static PPWorker pp_proc(nn_to_pp_chan.reader(), pp_to_disp_chan.writer());
  static DPWorker dp_proc(pp_to_disp_chan.reader());

  Run(InParallel(isp_proc, nn_proc, nn_proc_2, npu_proc, pp_proc, dp_proc),
      ExecutionMode::StaticNetwork, NETWORK_PRIORITY);
  osThreadExit();
}

void csp_app_main_init(void) {
  osThreadAttr_t attr = {};
  attr.name       = "MainApp";
  attr.stack_mem  = mainAppStack;
  attr.stack_size = sizeof(mainAppStack);
  attr.cb_mem     = &mainAppControlBlock;
  attr.cb_size    = sizeof(mainAppControlBlock);
  attr.priority   = MAIN_APP_PRIORITY;
  if (osThreadNew(MainApp_Task, NULL, &attr) == NULL) {
    printf("ERROR: MainApp_Task creation failed!\r\n");
  }
}
