// CAN msgs we care about
#define CHERY_ACC 0x3A2
#define CHERY_LKAS_HUD 0x307
#define CHERY_LKAS 0x345
#define CHERY_ENGINE 0x3E
#define CHERY_BRAKE 0x29A

#define CHERY_WHEEL_SENSOR 0x316 // RX for vehicle speed
#define CHERY_ACC_DATA 0x3A5
#define CHERY_STEER_BUTTON 0x360

// CAN bus numbers
#define CHERY_MAIN 0U
#define CHERY_AUX 1U
#define CHERY_CAM 2U

const SteeringLimits CHERY_STEERING_LIMITS = {
    .max_steer = 800,
    .max_rate_up = 10,
    .max_rate_down = 25,
    .max_rt_delta = 300,
    .max_rt_interval = 250000,
    .driver_torque_factor = 1,
    .driver_torque_allowance = 15,
    .type = TorqueDriverLimited,
};

const CanMsg CHERY_TX_MSGS[] = {
    {CHERY_ACC, 0, 8},
    {CHERY_LKAS, 0, 8},
    {CHERY_LKAS_HUD, 0, 8},
    {CHERY_STEER_BUTTON, 0, 8},
    {CHERY_STEER_BUTTON, 2, 8},
};

RxCheck chery_rx_checks[] = {
    {.msg = {{CHERY_WHEEL_SENSOR, CHERY_MAIN, 8, .frequency = 50U}, {0}, {0}}},
    {.msg = {{CHERY_ENGINE, CHERY_MAIN, 48, .frequency = 100U}, {0}, {0}}},
    {.msg = {{CHERY_BRAKE, CHERY_MAIN, 8, .frequency = 50U}, {0}, {0}}},
    // {.msg = {{CHERY_ACC_DATA, CHERY_MAIN, 8, .frequency = 50U}, {0}, {0}}},
};

// track msgs coming from OP so that we know what CAM msgs to drop and what to forward
static void chery_rx_hook(const CANPacket_t *to_push)
{
  if ((int)GET_BUS(to_push) == CHERY_MAIN)
  {
    int addr = GET_ADDR(to_push);

    // if (addr == CHERY_ENGINE_DATA) {
    //   // sample speed: scale by 0.01 to get kph
    //   int speed = (GET_BYTE(to_push, 2) << 8) | GET_BYTE(to_push, 3);
    //   vehicle_moving = speed > 10; // moving when speed > 0.1 kph
    // }

    // if (addr == CHERY_STEER_TORQUE) {
    //   int torque_driver_new = GET_BYTE(to_push, 0) - 127U;
    //   // update array of samples
    //   update_sample(&torque_driver, torque_driver_new);
    // }

    // // enter controls on rising edge of ACC, exit controls on ACC off
    // if (addr == CHERY_CRZ_CTRL) {
    //   acc_main_on = GET_BIT(to_push, 17U);
    //   bool cruise_engaged = GET_BYTE(to_push, 0) & 0x8U;
    //   pcm_cruise_check(cruise_engaged);
    // }

    // if (addr == CHERY_ENGINE_DATA) {
    //   gas_pressed = (GET_BYTE(to_push, 4) || (GET_BYTE(to_push, 5) & 0xF0U));
    // }

    // if (addr == CHERY_PEDALS) {
    //   brake_pressed = (GET_BYTE(to_push, 0) & 0x10U);
    // }

    generic_rx_checks((addr == CHERY_LKAS));
  }
  controls_allowed = true;
}

static bool chery_tx_hook(const CANPacket_t *to_send)
{
  bool tx = true;
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);

  // Check if msg is sent on the main BUS
  if (bus == CHERY_MAIN)
  {

    if (addr == CHERY_LKAS)
    {
      // tx = false;
    }
  }

  return tx;
}

static int chery_fwd_hook(int bus, int addr)
{
  int bus_fwd = -1;

  if (bus == CHERY_MAIN)
  {
    bool block = (addr == 0xa111);
    if (!block)
    {
      bus_fwd = CHERY_CAM;
    }
  }
  else if (bus == CHERY_CAM)
  {

    // bool block = (addr == CHERY_LKAS) || (addr == CHERY_ACC) || (addr == CHERY_LKAS_HUD) || (addr == CHERY_LKAS_CMD) || (addr == 0x387) || (addr == 0x3fc) || (addr == CHERY_ACC_DATA);
    // bool block = (addr == CHERY_LKAS) || (addr == CHERY_ACC) || (addr == CHERY_LKAS_HUD) || (addr == CHERY_LKAS_HUD) || (addr == 0x345);
    // bool block = (addr == CHERY_LKAS) || (addr == CHERY_ACC) || (addr == CHERY_LKAS_HUD) || (addr == CHERY_LKAS_HUD) || (addr == 0x345) || (addr == 0x3dc) || (addr == 0x3de) || (addr == 0x3ed) || (addr == 0x3fa) || (addr == 0x4dd);
    bool block = (addr == CHERY_LKAS) || (addr == CHERY_ACC);
    // --|| (addr != CHERY_ACC) || (addr != CHERY_LKAS_HUD) || (addr != 0x387) || (addr != CHERY_ACC_DATA);
    if (!block)
    {
      bus_fwd = CHERY_MAIN;
      // print("  Address: 0x");
      // puth(addr);
      // print("\n");
    }
  }
  else
  {
    // don't fwd
  }

  return bus_fwd;
}

static safety_config chery_init(uint16_t param)
{
  UNUSED(param);
  return BUILD_SAFETY_CFG(chery_rx_checks, CHERY_TX_MSGS);
}

const safety_hooks chery_hooks = {
    .init = chery_init,
    .rx = chery_rx_hook,
    .tx = chery_tx_hook,
    .fwd = chery_fwd_hook,
};
