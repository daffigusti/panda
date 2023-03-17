// Globals
//int giraffe_forward_camera_volvo = 0;
int acc_active_prev_volvo = 0;
int acc_ped_val_prev = 0;
int volvo_desired_angle_last = 0;
float volvo_speed = 0;

// diagnostic msgs
#define MSG_DIAG_CEM 0x726
#define MSG_DIAG_PSCM 0x730
#define MSG_DIAG_FSM 0x764
#define MSG_DIAG_CVM 0x793
#define MSG_DIAG_BROADCAST 0x7df

// platform C1
// msg ids
#define MSG_BTNS_VOLVO_C1 0x10 // Steering wheel buttons
#define MSG_FSM0_VOLVO_C1 0x30 // ACC status message
#define MSG_FSM1_VOLVO_C1 0xd0 // LKA steering message
#define MSG_FSM2_VOLVO_C1 0x160
#define MSG_FSM3_VOLVO_C1 0x270
#define MSG_FSM4_VOLVO_C1 0x280
#define MSG_FSM5_VOLVO_C1 0x355
#define MSG_PSCM0_VOLVO_C1 0xe0
#define MSG_PSCM1_VOLVO_C1 0x125    // Steering
#define MSG_ACC_PEDAL_VOLVO_C1 0x55 // Gas pedal
#define MSG_SPEED_VOLVO_C1 0x130    // Speed signal

// safety params
const float DEG_TO_CAN_VOLVO_C1 = 1/0.04395;            // 22.75312855517634, inverse of dbc scaling
const int VOLVO_MAX_DELTA_OFFSET_ANGLE = 20/0.04395-1;  // max degrees divided by k factor in dbc 0.04395. -1 to give a little safety margin. 
                                                        // 25 degrees allowed, more will trigger disengage by servo.
const int VOLVO_MAX_ANGLE_REQ = 8189;                   // max, min angle req, set at 2steps from max and min values.
const int VOLVO_MIN_ANGLE_REQ = -8190;                  // 14 bits long, min -8192 -> 8191.

const struct lookup_t VOLVO_LOOKUP_ANGLE_RATE_UP = {
  {7., 17., 36.},  // 25.2, 61.2, 129.6 km/h
  {2, .25, .1}
};
const struct lookup_t VOLVO_LOOKUP_ANGLE_RATE_DOWN = {
  {7., 17., 36.},
  {2, .25, .1}
};

struct sample_t volvo_angle_meas;  // last 3 steer angles

// TX checks
// platform c1
const CanMsg VOLVO_C1_TX_MSGS[] = { {MSG_FSM0_VOLVO_C1, 0, 8}, {MSG_FSM1_VOLVO_C1, 0, 8},
                                     {MSG_FSM2_VOLVO_C1, 0, 8}, {MSG_FSM3_VOLVO_C1, 0, 8},
                                     {MSG_FSM4_VOLVO_C1, 0, 8},
                                     {MSG_BTNS_VOLVO_C1, 0, 8},
                                     {MSG_PSCM0_VOLVO_C1, 2, 8}, {MSG_PSCM1_VOLVO_C1, 2, 8},
                                     {MSG_DIAG_FSM, 2, 8}, {MSG_DIAG_PSCM, 0, 8},
                                     {MSG_DIAG_CEM, 0, 8}, {MSG_DIAG_CVM, 0, 8},
                                     {MSG_DIAG_BROADCAST, 0, 8}, {MSG_DIAG_BROADCAST, 2, 8},
                                    };

const int VOLVO_C1_TX_MSGS_LEN = sizeof(VOLVO_C1_TX_MSGS) / sizeof(VOLVO_C1_TX_MSGS[0]);

// expected_timestep in microseconds between messages.
AddrCheckStruct volvo_c1_checks[] = {
  {.msg = {{MSG_FSM0_VOLVO_C1,       2, 8, .check_checksum = false, .expected_timestep = 10000U}}},
  {.msg = {{MSG_FSM1_VOLVO_C1,       2, 8, .check_checksum = false, .expected_timestep = 20000U}}},
  {.msg = {{MSG_PSCM0_VOLVO_C1,      0, 8, .check_checksum = false, .expected_timestep = 20000U}}},
  {.msg = {{MSG_PSCM1_VOLVO_C1,      0, 8, .check_checksum = false, .expected_timestep = 20000U}}},
  {.msg = {{MSG_ACC_PEDAL_VOLVO_C1,  0, 8, .check_checksum = false, .expected_timestep = 20000U}}},
};

#define VOLVO_C1_RX_CHECKS_LEN sizeof(volvo_c1_checks) / sizeof(volvo_c1_checks[0])

addr_checks volvo_c1_rx_checks = {volvo_c1_checks, VOLVO_C1_RX_CHECKS_LEN};

static const addr_checks* volvo_c1_init(uint16_t param) {
  UNUSED(param);
  return &volvo_c1_rx_checks;
}

static int volvo_c1_rx_hook(CANPacket_t *to_push) {

  bool valid = addr_safety_check(to_push, &volvo_c1_rx_checks, NULL, NULL, NULL, NULL);

  if( valid ) {
    int bus = GET_BUS(to_push);
    int addr = GET_ADDR(to_push);

    // check acc status
    if( (addr == MSG_FSM0_VOLVO_C1) && (bus == 2) ) {
      //giraffe_forward_camera_volvo = 1;
      bool acc_active = (GET_BYTE(to_push, 7) & 0x04);

      // only allow lateral control when acc active
      if(acc_active && !acc_active_prev_volvo) {
        controls_allowed = 1;
      }
      acc_active_prev_volvo = acc_active;
    }

    if( bus == 0 ) {
      // Current steering angle
      if (addr == MSG_PSCM1_VOLVO_C1) {
        // 2bytes long.
        int angle_meas_new = (GET_BYTE(to_push, 5) << 8) | (GET_BYTE(to_push, 6));
        // Remove offset.
        angle_meas_new = angle_meas_new-32768;

        // update array of samples
        update_sample(&volvo_angle_meas, angle_meas_new);
      }

      // Get current speed
      if (addr == MSG_SPEED_VOLVO_C1) {
        // Factor 0.01
        volvo_speed = ((GET_BYTE(to_push, 3) << 8) | (GET_BYTE(to_push, 4))) * 0.01 / 3.6;
      }

//      // dont forward if message is on bus 0
//      if( addr == MSG_FSM0_VOLVO_C1 ) {
//        giraffe_forward_camera_volvo = 0;
//      }
//      // If LKA msg is on bus 0, then relay is unexpectedly closed
//      if( (safety_mode_cnt > RELAY_TRNS_TIMEOUT) && (addr == MSG_FSM1_VOLVO_C1) ) {
//        relay_malfunction_set();
//      }
    }
  }

  return valid;
}

static int volvo_c1_tx_hook(CANPacket_t *to_send) {

  int tx = 1;
  //int bus = GET_BUS(to_send);
  int addr = GET_ADDR(to_send);
  bool violation = 0;

  tx = msg_allowed(to_send, VOLVO_C1_TX_MSGS, VOLVO_C1_TX_MSGS_LEN);

  if ( addr == MSG_FSM1_VOLVO_C1 ) {
    int desired_angle = ((GET_BYTE(to_send, 4) & 0x3f) << 8) | (GET_BYTE(to_send, 5)); // 14 bits long
    bool lka_active = (GET_BYTE(to_send, 7) & 0x3) > 0;  // Steer direction bigger than 0, commanding lka to steer

    // remove offset
    desired_angle = desired_angle-8192;

    if (controls_allowed && lka_active) {
      // add 1 to not false trigger the violation
      float delta_angle_float;
      delta_angle_float = (interpolate(VOLVO_LOOKUP_ANGLE_RATE_UP, volvo_speed) * DEG_TO_CAN_VOLVO_C1) + 1.;
      int delta_angle_up = (int)(delta_angle_float);
      delta_angle_float =  (interpolate(VOLVO_LOOKUP_ANGLE_RATE_DOWN, volvo_speed) * DEG_TO_CAN_VOLVO_C1) + 1.;
      int delta_angle_down = (int)(delta_angle_float);
      int highest_desired_angle = volvo_desired_angle_last + ((volvo_desired_angle_last > 0) ? delta_angle_up : delta_angle_down);
      int lowest_desired_angle = volvo_desired_angle_last - ((volvo_desired_angle_last >= 0) ? delta_angle_down : delta_angle_up);

      // max request offset from actual angle
      int hi_angle_req = MIN(desired_angle + VOLVO_MAX_DELTA_OFFSET_ANGLE, VOLVO_MAX_ANGLE_REQ);
      int lo_angle_req = MAX(desired_angle - VOLVO_MAX_DELTA_OFFSET_ANGLE, VOLVO_MIN_ANGLE_REQ);

      // check for violation;
      violation |= max_limit_check(desired_angle, highest_desired_angle, lowest_desired_angle);
      violation |= max_limit_check(desired_angle, hi_angle_req, lo_angle_req);
    }
    volvo_desired_angle_last = desired_angle;

    // desired steer angle should be the same as steer angle measured when controls are off
    // dont check when outside of measurable range. desired_angle can only be -8192->8191 (+-360).
    if ((!controls_allowed)
          && ((volvo_angle_meas.min - 1) >= VOLVO_MAX_ANGLE_REQ)
          && ((volvo_angle_meas.max + 1) <= VOLVO_MIN_ANGLE_REQ)
          && ((desired_angle < (volvo_angle_meas.min - 1)) || (desired_angle > (volvo_angle_meas.max + 1)))) {
      violation = 1;
    }

    // no lka_enabled bit if controls not allowed
    if (!controls_allowed && lka_active) {
      violation = 1;
    }
  }

  if (violation) {
    controls_allowed = 0;
    tx = 0;
  }

  return tx;
}

static int volvo_c1_fwd_hook(int bus_num, CANPacket_t *to_fwd) {

  int bus_fwd = -1; // fallback to do not forward
  int addr = GET_ADDR(to_fwd);

//  if(giraffe_forward_camera_volvo) {
    if( bus_num == 0 ){
      bool block_msg = (addr == MSG_PSCM1_VOLVO_C1);
      if (!block_msg) {
        bus_fwd = 2; // forward 0 -> 2
      }
    }

    if( bus_num == 2 ) {
      bool block_msg = (addr == MSG_FSM1_VOLVO_C1); // block if lkas msg
      if(!block_msg) {
        bus_fwd = 0; // forward bus 2 -> 0
      }
    }
//  }
  return bus_fwd;
}

const safety_hooks volvo_c1_hooks = {
  .init = volvo_c1_init,
  .rx = volvo_c1_rx_hook,
  .tx = volvo_c1_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = volvo_c1_fwd_hook,
};
