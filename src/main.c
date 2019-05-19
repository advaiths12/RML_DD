//! \defgroup PROJ_LAB13F_OVERVIEW Project Overview
//!
//! Basic implementation of FOC by using the estimator for angle and speed
//! feedback only.  Adds in SpinTAC Position Contol and SpinTAC Position Move
//!	Dual position control using SpinTAC

// **************************************************************************
// the includes

// system includes
#include "main_position_2mtr.h"
#include "mdrive.h"

#ifdef FLASH
#pragma CODE_SECTION(motor1_ISR, "ramfuncs");
#pragma CODE_SECTION(motor2_ISR, "ramfuncs");
#pragma CODE_SECTION(EEPROM_erase, "ramfuncs");
#pragma CODE_SECTION(EEPROM_write, "ramfuncs");

#endif

// Include header files used in the main function

// **************************************************************************
// the defines

// **************************************************************************
// the globals
//
//

KINEMATICS_Handle kinematicsHandle; //!< the handle for the kinematics object
//!< transform
KINEMATICS_Obj kinematics; //!< the kinematics obj

CLARKE_Handle clarkeHandle_I[2]; //!< the handle for the current Clarke
//!< transform
CLARKE_Obj clarke_I[2]; //!< the current Clarke transform object

PARK_Handle parkHandle[2]; //!< the handle for the current Parke
//!< transform
PARK_Obj park[2]; //!< the current Parke transform object

CLARKE_Handle clarkeHandle_V[2]; //!< the handle for the voltage Clarke
//!< transform
CLARKE_Obj clarke_V[2]; //!< the voltage Clarke transform object

PID_Obj pid[2][2]; //!< three objects for PID controllers
//!< 0 - Id, 1 - Iq
PID_Handle pidHandle[2][2]; //!< three handles for PID controllers
//!< 0 - Id, 1 - Iq

_iq gGains[2][2];
_iq gCartesianGains[2][2];

IPARK_Handle iparkHandle[2]; //!< the handle for the inverse Park
//!< transform
IPARK_Obj ipark[2]; //!< the inverse Park transform object

SVGEN_Handle svgenHandle[2]; //!< the handle for the space vector generator
SVGEN_Obj svgen[2]; //!< the space vector generator object

HAL_Handle halHandle; //!< the handle for the hardware abstraction
    //!< layer for common CPU setup
HAL_Obj hal; //!< the hardware abstraction layer object

HAL_Handle_mtr halHandleMtr[2]; //!< the handle for the hardware abstraction
//!< layer specific to the motor board.
HAL_Obj_mtr halMtr[2]; //!< the hardware abstraction layer object
//!< specific to the motor board.




HAL_PwmData_t gPwmData[2] = {
    { _IQ(0.0), _IQ(0.0), _IQ(0.0) }, //!< contains the
    { _IQ(0.0), _IQ(0.0), _IQ(0.0) }
}; //!< pwm values for each phase.
//!< -1.0 is 0%, 1.0 is 100%

HAL_AdcData_t gAdcData[2]; //!< contains three current values, three
//!< voltage values and one DC buss value

MATH_vec3 gOffsets_I_pu[2] = {
    { _IQ(0.0), _IQ(0.0), _IQ(0.0) }, //!< contains
    { _IQ(0.0), _IQ(0.0), _IQ(0.0) }
}; //!< the offsets for the current feedback

MATH_vec3 gOffsets_V_pu[2] = {
    { _IQ(0.0), _IQ(0.0), _IQ(0.0) }, //!< contains
    { _IQ(0.0), _IQ(0.0), _IQ(0.0) }
}; //!< the offsets for the voltage feedback

MATH_vec2 gIdq_ref_pu[2] = { { _IQ(0.0), _IQ(0.0) }, //!< contains the Id and
    { _IQ(0.0), _IQ(0.0) } }; //!< Iq references

MATH_vec2 gVdq_out_pu[2] = {
    { _IQ(0.0), _IQ(0.0) }, //!< contains the output
    { _IQ(0.0), _IQ(0.0) }
}; //!< Vd and Vq from the current controllers

MATH_vec2 gIdq_pu[2] = { { _IQ(0.0), _IQ(0.0) }, //!< contains the Id and Iq
    { _IQ(0.0), _IQ(0.0) } }; //!< measured values

FILTER_FO_Handle filterHandle[2][6]; //!< the handles for the 3-current and
//!< 3-voltage filters for offset calculation
FILTER_FO_Obj filter[2][6]; //!< the 3-current and 3-voltage filters for offset
//!< calculation
uint32_t gOffsetCalcCount[2] = { 0, 0 };

USER_Params gUserParams[2];

uint32_t gAlignCount[2] = { 0, 0 };

int ECANIDS = 6;

ECAN_Mailbox gECAN_Mailbox;
FIFO_ID_Obj gECAN_rxFIFO_ID;
FIFO_ID_Obj gECAN_txFIFO_ID;

uint64_t msg_data;

volatile MOTOR_Vars_t gMotorVars[2] = {
    MOTOR_Vars_INIT_Mtr1, MOTOR_Vars_INIT_Mtr2
}; //!< the global motor
//!< variables that are defined in main.h and
//!< used for display in the debugger's watch
//!< window
volatile float gCommand[2] = { 0, 0 };
volatile _iq gMotorPosition[2] = { 0, 0 };
volatile _iq gError[2] = { 0, 0 };

#ifdef FLASH
// Used for running BackGround in flash, and ISR in RAM
extern uint16_t *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;
#endif

#ifdef DRV8301_SPI
// Watch window interface to the 8301 SPI
DRV_SPI_8301_Vars_t gDrvSpi8301Vars[2];
#endif

_iq gFlux_pu_to_Wb_sf[2];

_iq gFlux_pu_to_VpHz_sf[2];

_iq gTorque_Ls_Id_Iq_pu_to_Nm_sf[2];

_iq gTorque_Flux_Iq_pu_to_Nm_sf[2];

_iq gSpeed_krpm_to_pu_sf[2];

_iq gSpeed_pu_to_krpm_sf[2];

_iq gSpeed_hz_to_krpm_sf[2];

_iq gCurrent_A_to_pu_sf[2];
_iq theta1, theta2;
_iq jacobian[2][2];

// **************************************************************************
// the functions

static inline uint32_t reverse_byte_order(uint32_t value)
{
    return (value & 0x000000ff) << 24 | (value & 0x0000ff00) << 8 | (value & 0x00ff0000) >> 8 | (value & 0xff000000) >> 24;
}

void main(void)
{
    // IMPORTANT NOTE: If you are not familiar with MotorWare coding guidelines
    // please refer to the following document:
    // C:/ti/motorware/motorware_1_01_00_1x/docs/motorware_coding_standards.pdf

    // Only used if running from FLASH
    // Note that the variable FLASH is defined by the project

#ifdef FLASH
    // Copy time critical code and Flash setup code to RAM
    // The RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
    // symbols are created by the linker. Refer to the linker files.
    memCopy((uint16_t*)&RamfuncsLoadStart, (uint16_t*)&RamfuncsLoadEnd,
        (uint16_t*)&RamfuncsRunStart);
    memCopy(&Flash28_API_LoadStart, &Flash28_API_LoadEnd, &Flash28_API_RunStart);
#endif

    mdrive_init();
    // Begin the background loop
    for (;;) {

        // Waiting for enable system flag to be set
        // Motor 1 Flag_enableSys is the master control.
        while (!(gMotorVars[HAL_MTR1].Flag_enableSys))
            ;

        // loop while the enable system flag is true
        // Motor 1 Flag_enableSys is the master control.
        while (gMotorVars[HAL_MTR1].Flag_enableSys) {
            gMotorPosition[HAL_MTR1] = HAL_getEncoderMechanicalAngle(halHandleMtr[HAL_MTR1]);
            gMotorPosition[HAL_MTR2] = HAL_getEncoderMechanicalAngle(halHandleMtr[HAL_MTR2]);
            // Send CAN messages
            while (!(ECAN_sendMsg_FIFO_ID_One(halHandle->ecanaHandle, &gECAN_Mailbox,
                &gECAN_txFIFO_ID)))
                ;
            FIFO_FLUSH(gECAN_txFIFO_ID);
            // Check if CAN messages are available
            if (ECAN_checkMail(halHandle->ecanaHandle)) {
                ECAN_getMsgFIFO_ID_N(halHandle->ecanaHandle, &gECAN_Mailbox,
                    &gECAN_rxFIFO_ID);
                ECAN_clearCANRMP(halHandle->ecanaHandle);
            }

            // Process the received CAN messages
            while (!(FIFO_IS_EMPTY(gECAN_rxFIFO_ID))) {
                MSG_t msg_temp = FIFO_FRONT(gECAN_rxFIFO_ID);
                FIFO_POP(gECAN_rxFIFO_ID);
                msg_data = (((uint64_t)reverse_byte_order(msg_temp.dataH)) << 32) | reverse_byte_order(msg_temp.dataL);
                unpack_message(msg_temp.msgID & 0x7f0, msg_data, msg_temp.dataLength);

                // Temp variables
                float offset_data; //ADC offsets
                uint16_t mech_offset; //mech offsets
                uint16_t elec_offset; //elec offsets
                uint16_t direction; //encoder direction
                uint16_t commutation; //commutation direction
                float gains[2][2]; // position and current gains
                float kp; // kp gains
                float ki; //ki gains
                _iq temp_kp, temp_ki, temp_kd;
                switch (msg_temp.msgID & 0x7f0) {
                    //          case 0x400:
                    //            HAL_disableGlobalInts(halHandle);
                    //            //HAL_disableDebugInt(halHandle);
                    //            PIE_disable(halHandle->pieHandle);
                    //
                    //            PERSIST_erase();
                    //            PERSIST_saveAll();
                    //            //PERSIST_depletionRecovery();
                    //            mdrive_init();
                    //            break;
                    //          case 0x300:
                    //            PERSIST_loadAll();
                    //            break;
                case 0x560: // Setup motor1 encoder parameters
                    decode_can_0x560_mech_offset(&can_0x560_motor1_encoder_data,
                        &mech_offset);
                    decode_can_0x560_elec_offset(&can_0x560_motor1_encoder_data,
                        &elec_offset);
                    decode_can_0x560_direction(&can_0x560_motor1_encoder_data,
                        &direction);
                    decode_can_0x560_commutation(&can_0x560_motor1_encoder_data,
                        &commutation);
                    HAL_setEncoderDirection(halHandleMtr[HAL_MTR1], direction);
                    HAL_setCommutationDirection(halHandleMtr[HAL_MTR1], commutation);
                    HAL_setEncoderMechAngleOffset(halHandleMtr[HAL_MTR1], mech_offset);
                    HAL_setEncoderElecAngleOffset(halHandleMtr[HAL_MTR1], elec_offset);
                    break;

                case 0x540: // Setup motor2 encoder parameters
                    decode_can_0x540_mech_offset(&can_0x540_motor2_encoder_data,
                        &mech_offset);
                    decode_can_0x540_elec_offset(&can_0x540_motor2_encoder_data,
                        &elec_offset);
                    decode_can_0x540_direction(&can_0x540_motor2_encoder_data,
                        &direction);
                    decode_can_0x540_commutation(&can_0x540_motor2_encoder_data,
                        &commutation);
                    HAL_setEncoderDirection(halHandleMtr[HAL_MTR2], direction);
                    HAL_setCommutationDirection(halHandleMtr[HAL_MTR2], commutation);
                    HAL_setEncoderMechAngleOffset(halHandleMtr[HAL_MTR2], mech_offset);
                    HAL_setEncoderElecAngleOffset(halHandleMtr[HAL_MTR2], elec_offset);
                    break;

                case 0x520: // Setup position gains
                    decode_can_0x520_motor1_Kp(&can_0x520_position_gains_data,
                        &gains[0][0]);
                    decode_can_0x520_motor1_Kd(&can_0x520_position_gains_data,
                        &gains[0][1]);
                    decode_can_0x520_motor2_Kp(&can_0x520_position_gains_data,
                        &gains[1][0]);
                    decode_can_0x520_motor2_Kd(&can_0x520_position_gains_data,
                        &gains[1][1]);
                    gGains[0][0] = _IQ(gains[0][0]);
                    gGains[0][1] = _IQ(gains[0][1]);
                    gGains[1][0] = _IQ(gains[1][0]);
                    gGains[1][1] = _IQ(gains[1][1]);
                    break;
                case 0x480: // Setup current id gains
                    decode_can_0x480_motor1_Kp(&can_0x480_current_id_gains_data, &kp);
                    decode_can_0x480_motor1_Ki(&can_0x480_current_id_gains_data, &ki);
                    PID_setGains(pidHandle[HAL_MTR1][0], _IQ(kp), _IQ(ki), _IQ(0.0));
                    PID_setGains(pidHandle[HAL_MTR1][1], _IQ(kp), _IQ(ki), _IQ(0.0));
                    decode_can_0x480_motor2_Kp(&can_0x480_current_id_gains_data, &kp);
                    decode_can_0x480_motor2_Ki(&can_0x480_current_id_gains_data, &ki);
                    // P term = Kp_Iq, I term = Ki_Iq, D term = 0
                    PID_setGains(pidHandle[HAL_MTR2][0], _IQ(kp), _IQ(ki), _IQ(0.0));
                    PID_setGains(pidHandle[HAL_MTR2][1], _IQ(kp), _IQ(ki), _IQ(0.0));
                    break;


                case 0x200:
                    decode_can_0x200_motor_1(&can_0x200_setpoint_data,
                        (float*)&gCommand[HAL_MTR1]);
                    decode_can_0x200_motor_2(&can_0x200_setpoint_data,
                        (float*)&gCommand[HAL_MTR2]);
                    break;
                case 0x600: //Get all parameters
                    // Position Gains
                    encode_can_0x520_motor1_Kp(&can_0x520_position_gains_data,
                        _IQtoF(gGains[HAL_MTR1][0]));
                    encode_can_0x520_motor1_Kd(&can_0x520_position_gains_data,
                        _IQtoF(gGains[HAL_MTR1][1]));
                    encode_can_0x520_motor2_Kp(&can_0x520_position_gains_data,
                        _IQtoF(gGains[HAL_MTR2][0]));
                    encode_can_0x520_motor2_Kd(&can_0x520_position_gains_data,
                        _IQtoF(gGains[HAL_MTR2][1]));
                    pack_message(0x520, &msg_data);
                    send_msg(0x520, msg_data, DLC_8);

                    // Current I_d gains
                    PID_getGains(pidHandle[HAL_MTR1][0], &temp_kp, &temp_ki, &temp_kd);
                    encode_can_0x480_motor1_Kp(&can_0x480_current_id_gains_data,
                        _IQtoF(temp_kp));
                    encode_can_0x480_motor1_Ki(&can_0x480_current_id_gains_data,
                        _IQtoF(temp_ki));
                    PID_getGains(pidHandle[HAL_MTR2][0], &temp_kp, &temp_ki, &temp_kd);
                    encode_can_0x480_motor2_Kp(&can_0x480_current_id_gains_data,
                        _IQtoF(temp_kp));
                    encode_can_0x480_motor2_Ki(&can_0x480_current_id_gains_data,
                        _IQtoF(temp_ki));
                    pack_message(0x480, &msg_data);
                    send_msg(0x480, msg_data, DLC_8);

                    // Current I_q gains
                    PID_getGains(pidHandle[HAL_MTR1][1], &temp_kp, &temp_ki, &temp_kd);
                    encode_can_0x500_motor1_Kp(&can_0x500_current_iq_gains_data,
                        _IQtoF(temp_kp));
                    encode_can_0x500_motor1_Ki(&can_0x500_current_iq_gains_data,
                        _IQtoF(temp_ki));
                    PID_getGains(pidHandle[HAL_MTR2][1], &temp_kp, &temp_ki, &temp_kd);
                    encode_can_0x500_motor2_Kp(&can_0x500_current_iq_gains_data,
                        _IQtoF(temp_kp));
                    encode_can_0x500_motor2_Ki(&can_0x500_current_iq_gains_data,
                        _IQtoF(temp_ki));
                    pack_message(0x500, &msg_data);
                    send_msg(0x500, msg_data, DLC_8);
                    // Mechanical Offsets
                    mech_offset = HAL_getEncoderMechAngleOffset(halHandleMtr[HAL_MTR1]);
                    elec_offset = HAL_getEncoderElecAngleOffset(halHandleMtr[HAL_MTR1]);
                    direction = HAL_getEncoderDirection(halHandleMtr[HAL_MTR1]);
                    commutation = HAL_getCommutationDirection(halHandleMtr[HAL_MTR1]);
                    encode_can_0x560_mech_offset(&can_0x560_motor1_encoder_data,
                        mech_offset);
                    encode_can_0x560_elec_offset(&can_0x560_motor1_encoder_data,
                        elec_offset);
                    encode_can_0x560_direction(&can_0x560_motor1_encoder_data, direction);
                    encode_can_0x560_commutation(&can_0x560_motor1_encoder_data,
                        commutation);
                    pack_message(0x560, &msg_data);
                    send_msg(0x560, msg_data, DLC_5);
                    mech_offset = HAL_getEncoderMechAngleOffset(halHandleMtr[HAL_MTR2]);
                    elec_offset = HAL_getEncoderElecAngleOffset(halHandleMtr[HAL_MTR2]);
                    direction = HAL_getEncoderDirection(halHandleMtr[HAL_MTR2]);
                    commutation = HAL_getCommutationDirection(halHandleMtr[HAL_MTR2]);
                    encode_can_0x540_mech_offset(&can_0x540_motor2_encoder_data,
                        mech_offset);
                    encode_can_0x540_elec_offset(&can_0x540_motor2_encoder_data,
                        elec_offset);
                    encode_can_0x540_direction(&can_0x540_motor2_encoder_data, direction);
                    encode_can_0x540_commutation(&can_0x540_motor2_encoder_data,
                        commutation);
                    pack_message(0x540, &msg_data);
                    send_msg(0x540, msg_data, DLC_5);
                    // Voltage and current offsets
                    // Voltage M1
                    encode_can_0x320_A(&can_0x320_motor1_V_offsets_data, 
                        _IQtoF(gOffsets_V_pu[HAL_MTR1].value[0]));
                    encode_can_0x320_B(&can_0x320_motor1_V_offsets_data, 
                        _IQtoF(gOffsets_V_pu[HAL_MTR1].value[1]));
                    encode_can_0x320_C(&can_0x320_motor1_V_offsets_data, 
                        _IQtoF(gOffsets_V_pu[HAL_MTR1].value[2]));
                    pack_message(0x320, &msg_data);
                    send_msg(0x320, msg_data, DLC_6);
                    //Current M1
                    encode_can_0x340_A(&can_0x340_motor1_I_offsets_data, 
                        _IQtoF(gOffsets_I_pu[HAL_MTR1].value[0]));
                    encode_can_0x340_B(&can_0x340_motor1_I_offsets_data, 
                        _IQtoF(gOffsets_I_pu[HAL_MTR1].value[1]));
                    encode_can_0x340_C(&can_0x340_motor1_I_offsets_data, 
                        _IQtoF(gOffsets_I_pu[HAL_MTR1].value[2]));
                    pack_message(0x340, &msg_data);
                    send_msg(0x340, msg_data, DLC_6);
                    // Voltage M2
                    encode_can_0x360_A(&can_0x360_motor2_V_offsets_data, 
                        _IQtoF(gOffsets_V_pu[HAL_MTR2].value[0]));
                    encode_can_0x360_B(&can_0x360_motor2_V_offsets_data, 
                        _IQtoF(gOffsets_V_pu[HAL_MTR2].value[1]));
                    encode_can_0x360_C(&can_0x360_motor2_V_offsets_data, 
                        _IQtoF(gOffsets_V_pu[HAL_MTR2].value[2]));
                    pack_message(0x360, &msg_data);
                    send_msg(0x360, msg_data, DLC_6);
                    //Current M2
                    encode_can_0x380_A(&can_0x380_motor2_I_offsets_data, 
                        _IQtoF(gOffsets_I_pu[HAL_MTR2].value[0]));
                    encode_can_0x380_B(&can_0x380_motor2_I_offsets_data, 
                        _IQtoF(gOffsets_I_pu[HAL_MTR2].value[1]));
                    encode_can_0x380_C(&can_0x380_motor2_I_offsets_data, 
                        _IQtoF(gOffsets_I_pu[HAL_MTR2].value[2]));
                    pack_message(0x380, &msg_data);
                    send_msg(0x380, msg_data, DLC_6);
                    break;
                case 0x320: // Setup Voltage offsets for motor 1
                    decode_can_0x320_A(&can_0x320_motor1_V_offsets_data, &offset_data);
                    gOffsets_V_pu[HAL_MTR1].value[0] = _IQ(offset_data);
                    decode_can_0x320_B(&can_0x320_motor1_V_offsets_data, &offset_data);
                    gOffsets_V_pu[HAL_MTR1].value[1] = _IQ(offset_data);
                    decode_can_0x320_C(&can_0x320_motor1_V_offsets_data, &offset_data);
                    gOffsets_V_pu[HAL_MTR1].value[2] = _IQ(offset_data);
                case 0x340: // Setup Current offsets for motor 1
                    decode_can_0x340_A(&can_0x340_motor1_I_offsets_data, &offset_data);
                    gOffsets_I_pu[HAL_MTR1].value[0] = _IQ(offset_data);
                    decode_can_0x340_B(&can_0x340_motor1_I_offsets_data, &offset_data);
                    gOffsets_I_pu[HAL_MTR1].value[1] = _IQ(offset_data);
                    decode_can_0x340_C(&can_0x340_motor1_I_offsets_data, &offset_data);
                    gOffsets_I_pu[HAL_MTR1].value[2] = _IQ(offset_data);
                case 0x360: // Setup Voltage offsets for motor 2
                    decode_can_0x360_A(&can_0x360_motor2_V_offsets_data, &offset_data);
                    gOffsets_V_pu[HAL_MTR2].value[0] = _IQ(offset_data);
                    decode_can_0x360_B(&can_0x360_motor2_V_offsets_data, &offset_data);
                    gOffsets_V_pu[HAL_MTR2].value[1] = _IQ(offset_data);
                    decode_can_0x360_C(&can_0x360_motor2_V_offsets_data, &offset_data);
                    gOffsets_V_pu[HAL_MTR2].value[2] = _IQ(offset_data);
                case 0x380: // Setup Current offsets for motor 2
                    decode_can_0x380_A(&can_0x380_motor2_I_offsets_data, &offset_data);
                    gOffsets_I_pu[HAL_MTR2].value[0] = _IQ(offset_data);
                    decode_can_0x380_B(&can_0x380_motor2_I_offsets_data, &offset_data);
                    gOffsets_I_pu[HAL_MTR2].value[1] = _IQ(offset_data);
                    decode_can_0x380_C(&can_0x380_motor2_I_offsets_data, &offset_data);
                    gOffsets_I_pu[HAL_MTR2].value[2] = _IQ(offset_data);
                default:
                    break;
                }
            }
            FIFO_FLUSH(gECAN_rxFIFO_ID);



            uint_least8_t mtrNum = HAL_MTR1;
            if (*gMotorVars[mtrNum].Flag_cartesian_control)
                cartesianControlComp();

            for (mtrNum = HAL_MTR1; mtrNum <= HAL_MTR2; mtrNum++) {


                // If Flag_enableSys is set AND Flag_Run_Identify is set THEN
                // enable PWMs and set the speed reference
                if (*(gMotorVars[mtrNum].Flag_Run_Identify) || *(gMotorVars[mtrNum].Flag_enableOffsetcalc)) {

                    // enable the PWM
                    HAL_enablePwm(halHandleMtr[mtrNum]);

                } else { // Flag_enableSys is set AND Flag_Run_Identify is not set

                    // disable the PWM
                    HAL_disablePwm(halHandleMtr[mtrNum]);

                    // clear integrator outputs
                    PID_setUi(pidHandle[mtrNum][0], _IQ(0.0));
                    PID_setUi(pidHandle[mtrNum][1], _IQ(0.0));

                    // clear Id and Iq references
                    gIdq_ref_pu[mtrNum].value[0] = _IQ(0.0);
                    gIdq_ref_pu[mtrNum].value[1] = _IQ(0.0);
                }
            } // end of for loop
        } // end of while(gFlag_enableSys) condition

        // disable the PWM
        HAL_disablePwm(halHandleMtr[HAL_MTR1]);
        HAL_disablePwm(halHandleMtr[HAL_MTR2]);

        *(gMotorVars[HAL_MTR1].Flag_Run_Identify) = 0;
        *(gMotorVars[HAL_MTR2].Flag_Run_Identify) = 0;

    } // end of for(;;) loop
} // end of main() function

inline _iq wrapToOne(_iq error)
{
    if (_IQabs(error) > _IQ(0.5))
        error += (error > _IQ(0.0)) ? -_IQ(1.0) : _IQ(1.0);
    return error;
}

inline _iq motor_ISR(HAL_MtrSelect_e mtrNum, _iq* angle_pu, _iq* mech_angle_pu,
    unsigned char* encoderState, _iq* speed)
{
    MATH_vec2 Iab_pu;
    MATH_vec2 Vab_pu;
    MATH_vec2 phasor;
    _iq refValue;
    _iq fbackValue;
    _iq outMax_pu;

    if (!*encoderState) {
        HAL_beginEncoderRead(halHandleMtr[mtrNum]);
        *encoderState = 1;
    } else {
        HAL_endEncoderRead(halHandleMtr[mtrNum]);
        // Run speed computation
        _iq new_mech_angle_pu = HAL_getEncoderMechanicalAngle(halHandleMtr[mtrNum]);
        *speed = wrapToOne(new_mech_angle_pu - *mech_angle_pu);
        *mech_angle_pu = new_mech_angle_pu;
        *encoderState = 0;
    }

    // acknowledge the ADC interrupt
    switch (mtrNum) {
    case HAL_MTR1:
        HAL_acqAdcInt(halHandle, ADC_IntNumber_1);
        break;
    case HAL_MTR2:
        HAL_acqAdcInt(halHandle, ADC_IntNumber_2);
        break;
    }
    // convert the ADC data
    HAL_readAdcDataWithOffsets(halHandle, halHandleMtr[mtrNum],
        &gAdcData[mtrNum]);
    // remove offsets
    gAdcData[mtrNum].I.value[0] = gAdcData[mtrNum].I.value[0] - gOffsets_I_pu[mtrNum].value[0];
    gAdcData[mtrNum].I.value[1] = gAdcData[mtrNum].I.value[1] - gOffsets_I_pu[mtrNum].value[1];
    gAdcData[mtrNum].I.value[2] = gAdcData[mtrNum].I.value[2] - gOffsets_I_pu[mtrNum].value[2];
    gAdcData[mtrNum].V.value[0] = gAdcData[mtrNum].V.value[0] - gOffsets_V_pu[mtrNum].value[0];
    gAdcData[mtrNum].V.value[1] = gAdcData[mtrNum].V.value[1] - gOffsets_V_pu[mtrNum].value[1];
    gAdcData[mtrNum].V.value[2] = gAdcData[mtrNum].V.value[2] - gOffsets_V_pu[mtrNum].value[2];

    // Check if we need to flip the direction of commutation.
    if (!HAL_getCommutationDirection(halHandleMtr[mtrNum])) {
        _iq tempI = gAdcData[mtrNum].I.value[0];
        gAdcData[mtrNum].I.value[0] = gAdcData[mtrNum].I.value[1];
        gAdcData[mtrNum].I.value[1] = tempI;
        _iq tempV = gAdcData[mtrNum].V.value[0];
        gAdcData[mtrNum].V.value[0] = gAdcData[mtrNum].V.value[1];
        gAdcData[mtrNum].V.value[1] = tempV;
    }

    // run Clarke transform on current.  Three values are passed, two values
    // are returned.
    CLARKE_run(clarkeHandle_I[mtrNum], &gAdcData[mtrNum].I, &Iab_pu);

    // compute the sine and cosine phasor values which are part of the
    // Park transform calculations. Once these values are computed,
    // they are copied into the PARK module, which then uses them to
    // transform the voltages from Alpha/Beta to DQ reference frames.
    phasor.value[0] = _IQcosPU(*angle_pu);
    phasor.value[1] = _IQsinPU(*angle_pu);

    // set the phasor in the Park transform
    PARK_setPhasor(parkHandle[mtrNum], &phasor);

    // Run the Park module.  This converts the current vector from
    // stationary frame values to synchronous frame values.
    PARK_run(parkHandle[mtrNum], &Iab_pu, &gIdq_pu[mtrNum]);

    // run the appropriate controller
    if (*(gMotorVars[mtrNum].Flag_Run_Identify)) {
        // Declaration of local variables.

        // check if the motor should run position controller or be forced into encoder alignment
        if (!*(gMotorVars[mtrNum].Flag_enableAlignment) && !*(gMotorVars[mtrNum].Flag_enableMechAlignment)) {
            // generate the motor electrical angle and compute the speed

            *angle_pu = HAL_getEncoderElectricalAngle(halHandleMtr[mtrNum],
                USER_MOTOR_NUM_POLE_PAIRS);
            if (*gMotorVars[mtrNum].Flag_position_control) {
                // This is the position controller
                //Compute position and speed

                // Compute error command - feedback
                _iq error = _IQ(gCommand[mtrNum]) - *mech_angle_pu;
                // Wrap around 0 to 1.
                error = wrapToOne(error);
                gError[mtrNum] = error;

                // Set D-axis current to 0
                gIdq_ref_pu[mtrNum].value[0] = _IQ(0.0);
                // Set Q-axis current to command
                _iq command = _IQmpy(gGains[mtrNum][0], error) - _IQmpyI32(4500, _IQmpy(gGains[mtrNum][1], *speed));
                gIdq_ref_pu[mtrNum].value[1] = _IQdiv(command, _IQ(USER_IQ_FULL_SCALE_CURRENT_A));
            }
            if (*gMotorVars[mtrNum].Flag_cartesian_control){
              //TODO:
            
            }

            if (*gMotorVars[mtrNum].Flag_current_control) {
                gIdq_ref_pu[mtrNum].value[0] = _IQ(0.0);
                gIdq_ref_pu[mtrNum].value[1] = _IQdiv(_IQ(gCommand[mtrNum]), _IQ(USER_IQ_FULL_SCALE_CURRENT_A));
            }
        } else {
            // the electric alignment procedure is in effect
            if (*(gMotorVars[mtrNum].Flag_enableAlignment)) {
                // force motor angle and speed to 0
                *angle_pu = _IQ(0.0);

                // set D-axis current to Rs estimation current
                gIdq_ref_pu[mtrNum].value[0] = _IQ(USER_MOTOR_RES_EST_CURRENT / USER_IQ_FULL_SCALE_CURRENT_A);
                // set Q-axis current to 0
                gIdq_ref_pu[mtrNum].value[1] = _IQ(0.0);

                // save encoder reading when forcing motor into alignment
                HAL_setEncoderElecAngleOffset(
                    halHandleMtr[mtrNum], HAL_getEncoderPosition(halHandleMtr[mtrNum]));

                // if alignment counter exceeds threshold, exit alignment
                if (gAlignCount[mtrNum]++ >= gUserParams[mtrNum].ctrlWaitTime[CTRL_State_OffLine]) {
                    *(gMotorVars[mtrNum].Flag_enableAlignment) = 0;
                    *(gMotorVars[mtrNum].Flag_Run_Identify) = 0;
                    gAlignCount[mtrNum] = 0;
                    send_status_msg();
                    gIdq_ref_pu[mtrNum].value[0] = _IQ(0.0);
                }
            } else {
                // Calibrate Motor Zero
                HAL_setEncoderMechAngleOffset(
                    halHandleMtr[mtrNum], HAL_getEncoderPosition(halHandleMtr[mtrNum]));
                // if alignment counter exceeds threshold, exit alignment
                *(gMotorVars[mtrNum].Flag_enableMechAlignment) = 0;
                *(gMotorVars[mtrNum].Flag_Run_Identify) = 0;
                send_status_msg();
                gAlignCount[mtrNum] = 0;
            }
        }

        // Get the reference value for the d-axis current controller.
        refValue = gIdq_ref_pu[mtrNum].value[0];

        // Get the actual value of Id
        fbackValue = gIdq_pu[mtrNum].value[0];

        // The next instruction executes the PI current controller for the
        // d axis and places its output in Vdq_pu.value[0], which is the
        // control voltage along the d-axis (Vd)
        PID_run(pidHandle[mtrNum][0], refValue, fbackValue,
            &(gVdq_out_pu[mtrNum].value[0]));

        // get the Iq reference value
        refValue = gIdq_ref_pu[mtrNum].value[1];

        // get the actual value of Iq
        fbackValue = gIdq_pu[mtrNum].value[1];

        // The voltage limits on the output of the q-axis current controller
        // are dynamic, and are dependent on the output voltage from the d-axis
        // current controller.  In other words, the d-axis current controller
        // gets first dibs on the available voltage, and the q-axis current
        // controller gets what's left over.  That is why the d-axis current
        // controller executes first. The next instruction calculates the
        // maximum limits for this voltage as:
        // Vq_min_max = +/- sqrt(Vbus^2 - Vd^2)
        outMax_pu = _IQsqrt(
            _IQ(USER_MAX_VS_MAG_PU * USER_MAX_VS_MAG_PU) - _IQmpy(gVdq_out_pu[mtrNum].value[0], gVdq_out_pu[mtrNum].value[0]));

        // Set the limits to +/- outMax_pu
        PID_setMinMax(pidHandle[mtrNum][1], -outMax_pu, outMax_pu);

        // The next instruction executes the PI current controller for the
        // q axis and places its output in Vdq_pu.value[1], which is the
        // control voltage vector along the q-axis (Vq)
        PID_run(pidHandle[mtrNum][1], refValue, fbackValue,
            &(gVdq_out_pu[mtrNum].value[1]));

        // The voltage vector is now calculated and ready to be applied to the
        // motor in the form of three PWM signals.  However, even though the
        // voltages may be supplied to the PWM module now, they won't be
        // applied to the motor until the next PWM cycle.

        // compute the sine and cosine phasor values which are part of the inverse
        // Park transform calculations. Once these values are computed,
        // they are copied into the IPARK module, which then uses them to
        // transform the voltages from DQ to Alpha/Beta reference frames.
        phasor.value[0] = _IQcosPU(*angle_pu);
        phasor.value[1] = _IQsinPU(*angle_pu);

        // set the phasor in the inverse Park transform
        IPARK_setPhasor(iparkHandle[mtrNum], &phasor);

        // Run the inverse Park module.  This converts the voltage vector from
        // synchronous frame values to stationary frame values.
        IPARK_run(iparkHandle[mtrNum], &gVdq_out_pu[mtrNum], &Vab_pu);

        // These 3 statements compensate for variations in the DC bus by adjusting
        // the PWM duty cycle. The goal is to achieve the same volt-second product
        // regardless of the DC bus value.  To do this, we must divide the desired
        // voltage values by the DC bus value.  Or...it is easier to multiply by
        // 1/(DC bus value).
        _iq oneOverDcBus = _IQdiv(_IQ(1.0), gAdcData[mtrNum].dcBus);
        Vab_pu.value[0] = _IQmpy(Vab_pu.value[0], oneOverDcBus);
        Vab_pu.value[1] = _IQmpy(Vab_pu.value[1], oneOverDcBus);

        // Now run the space vector generator (SVGEN) module.
        // There is no need to do an inverse CLARKE transform, as this is
        // handled in the SVGEN_run function.
        SVGEN_run(svgenHandle[mtrNum], &Vab_pu, &(gPwmData[mtrNum].Tabc));
    } else if (*(gMotorVars[mtrNum].Flag_enableOffsetcalc)) {
        /* runOffsetsCalculation(mtrNum); */
        uint16_t cnt;
        // enable the PWM
        HAL_enablePwm(halHandleMtr[mtrNum]);

        for (cnt = 0; cnt < 3; cnt++) {
            // Set the PWMs to 50% duty cycle
            gPwmData[mtrNum].Tabc.value[cnt] = _IQ(0.0);

            // reset offsets used
            gOffsets_I_pu[mtrNum].value[cnt] = _IQ(0.0);
            gOffsets_V_pu[mtrNum].value[cnt] = _IQ(0.0);
            if (!HAL_getCommutationDirection(halHandleMtr[mtrNum])) {
                _iq tempI = gAdcData[mtrNum].I.value[0];
                gAdcData[mtrNum].I.value[0] = gAdcData[mtrNum].I.value[1];
                gAdcData[mtrNum].I.value[1] = tempI;
                _iq tempV = gAdcData[mtrNum].V.value[0];
                gAdcData[mtrNum].V.value[0] = gAdcData[mtrNum].V.value[1];
                gAdcData[mtrNum].V.value[1] = tempV;
            }

            // run offset estimation
            FILTER_FO_run(filterHandle[mtrNum][cnt], gAdcData[mtrNum].I.value[cnt]);
            FILTER_FO_run(filterHandle[mtrNum][cnt + 3], gAdcData[mtrNum].V.value[cnt]);
        }

        if (gOffsetCalcCount[mtrNum]++ >= gUserParams[mtrNum].ctrlWaitTime[CTRL_State_OffLine]) {
            *(gMotorVars[mtrNum].Flag_enableOffsetcalc) = 0;
            gOffsetCalcCount[mtrNum] = 0;

            for (cnt = 0; cnt < 3; cnt++) {
                // get calculated offsets from filter
                gOffsets_I_pu[mtrNum].value[cnt] = FILTER_FO_get_y1(filterHandle[mtrNum][cnt]);
                gOffsets_V_pu[mtrNum].value[cnt] = FILTER_FO_get_y1(filterHandle[mtrNum][cnt + 3]);

                // clear filters
                FILTER_FO_setInitialConditions(filterHandle[mtrNum][cnt], _IQ(0.0),
                    _IQ(0.0));
                FILTER_FO_setInitialConditions(filterHandle[mtrNum][cnt + 3], _IQ(0.0),
                    _IQ(0.0));
            }
            send_status_msg();
        }
    } else // *gMotorVars.Flag_Run_Identify = 0
    { //disable the PWM
        HAL_disablePwm(halHandleMtr[mtrNum]);

        // Set the PWMs to 50% duty cycle
        gPwmData[mtrNum].Tabc.value[0] = _IQ(0.0);
        gPwmData[mtrNum].Tabc.value[1] = _IQ(0.0);
        gPwmData[mtrNum].Tabc.value[2] = _IQ(0.0);
    }
    if (!HAL_getCommutationDirection(halHandleMtr[mtrNum])) {
        _iq temp = gPwmData[mtrNum].Tabc.value[0];
        gPwmData[mtrNum].Tabc.value[0] = gPwmData[mtrNum].Tabc.value[1];
        gPwmData[mtrNum].Tabc.value[1] = temp;
    }

    // write to the PWM compare registers, and then we are done!
    HAL_writePwmData(halHandleMtr[mtrNum], &gPwmData[mtrNum]);
    return fbackValue;
}

interrupt void motor1_ISR(void)
{
    static _iq angle_pu = _IQ(0.0);
    static _iq mech_angle_pu = _IQ(0.0);
    static _iq speed = _IQ(0.0);
    static uint32_t can_thread = 0;
    static unsigned char encoderState = 0;

    _iq fbackValue = motor_ISR(HAL_MTR1, &angle_pu, &mech_angle_pu, &encoderState, &speed);
    fbackValue = _IQmpy(fbackValue,_IQ(USER_IQ_FULL_SCALE_CURRENT_A));
    if (can_thread++ > 20) {
        uint64_t data;
        can_thread = 0;
        encode_can_0x240_position(&can_0x240_motor1_feedback_data,
            _IQtoF(mech_angle_pu));
        encode_can_0x240_velocity(&can_0x240_motor1_feedback_data,
            _IQtoF(speed));
        encode_can_0x240_effort(&can_0x240_motor1_feedback_data,
            _IQtoF(fbackValue));
        pack_message(0x240, &data);
        send_msg(0x240, data, DLC_6);
    }

    return;
}
interrupt void motor2_ISR(void)
{
    static _iq angle_pu = _IQ(0.0);
    static _iq mech_angle_pu = _IQ(0.0);
    static _iq speed = _IQ(0.0);
    static uint32_t can_thread = 0;
    static unsigned char encoderState = 0;

    _iq fbackValue = motor_ISR(HAL_MTR2, &angle_pu, &mech_angle_pu, &encoderState, &speed);
    fbackValue = _IQmpy(fbackValue,_IQ(USER_IQ_FULL_SCALE_CURRENT_A));
    if (can_thread++ > 20) {
        uint64_t data;
        can_thread = 0;
        encode_can_0x230_position(&can_0x230_motor2_feedback_data,
            _IQtoF(mech_angle_pu));
        encode_can_0x230_velocity(&can_0x230_motor2_feedback_data,
            _IQtoF(speed));
        encode_can_0x230_effort(&can_0x230_motor2_feedback_data,
            _IQtoF(fbackValue));
        pack_message(0x230, &data);
        send_msg(0x230, data, DLC_6);
    }

    return;
}

void pidSetup(HAL_MtrSelect_e mtrNum)
{
    // This equation uses the scaled maximum voltage vector, which is
    // already in per units, hence there is no need to include the #define
    // for USER_IQ_FULL_SCALE_VOLTAGE_V
    _iq maxVoltage_pu = _IQ(gUserParams[mtrNum].maxVsMag_pu * gUserParams[mtrNum].voltage_sf);

    _iq Kp_Id = USER_CURRENT_DAXIS_PGAIN;
    _iq Ki_Id = USER_CURRENT_DAXIS_IGAIN;
    _iq Kp_Iq = USER_CURRENT_QAXIS_PGAIN;
    _iq Ki_Iq = USER_CURRENT_QAXIS_IGAIN;

    // There are two PI controllers; two current
    // controllers.  Each PI controller has two coefficients; Kp and Ki.
    // So you have a total of four coefficients that must be defined.
    // This is for the Id current controller
    pidHandle[mtrNum][0] = PID_init(&pid[mtrNum][0], sizeof(pid[mtrNum][0]));
    // This is for the Iq current controller
    pidHandle[mtrNum][1] = PID_init(&pid[mtrNum][1], sizeof(pid[mtrNum][1]));

    // The following instructions load the parameters for the d-axis
    // current controller.
    // P term = Kp_Id, I term = Ki_Id, D term = 0
    PID_setGains(pidHandle[mtrNum][0], Kp_Id, Ki_Id, _IQ(0.0));
    // The following instructions load the parameters for the q-axis
    // current controller.
    // P term = Kp_Iq, I term = Ki_Iq, D term = 0
    PID_setGains(pidHandle[mtrNum][1], Kp_Iq, Ki_Iq, _IQ(0.0));

    // Largest negative voltage = -maxVoltage_pu, largest positive
    // voltage = maxVoltage_pu
    PID_setMinMax(pidHandle[mtrNum][0], -maxVoltage_pu, maxVoltage_pu);
    //
    // The largest negative voltage = 0 and the largest positive
    // voltage = 0.  But these limits are updated every single ISR before
    // actually executing the Iq controller. The limits depend on how much
    // voltage is left over after the Id controller executes. So having an
    // initial value of 0 does not affect Iq current controller execution.
    PID_setMinMax(pidHandle[mtrNum][1], _IQ(0.0), _IQ(0.0));

    // Set the initial condition value for the integrator output to 0
    PID_setUi(pidHandle[mtrNum][0], _IQ(0.0));
    PID_setUi(pidHandle[mtrNum][1], _IQ(0.0));

    // Setup the position controller
    gGains[mtrNum][0] = USER_POSITION_PGAIN;
    gGains[mtrNum][1] = USER_POSITION_DGAIN;
}

void setupClarke_I(CLARKE_Handle handle,
    const uint_least8_t numCurrentSensors)
{
    _iq alpha_sf, beta_sf;

    // initialize the Clarke transform module for current
    if (numCurrentSensors == 3) {
        alpha_sf = _IQ(MATH_ONE_OVER_THREE);
        beta_sf = _IQ(MATH_ONE_OVER_SQRT_THREE);
    } else if (numCurrentSensors == 2) {
        alpha_sf = _IQ(1.0);
        beta_sf = _IQ(MATH_ONE_OVER_SQRT_THREE);
    } else {
        alpha_sf = _IQ(0.0);
        beta_sf = _IQ(0.0);
    }

    // set the parameters
    CLARKE_setScaleFactors(handle, alpha_sf, beta_sf);
    CLARKE_setNumSensors(handle, numCurrentSensors);

    return;
} // end of setupClarke_I() function

void setupClarke_V(CLARKE_Handle handle,
    const uint_least8_t numVoltageSensors)
{
    _iq alpha_sf, beta_sf;

    // initialize the Clarke transform module for voltage
    if (numVoltageSensors == 3) {
        alpha_sf = _IQ(MATH_ONE_OVER_THREE);
        beta_sf = _IQ(MATH_ONE_OVER_SQRT_THREE);
    } else {
        alpha_sf = _IQ(0.0);
        beta_sf = _IQ(0.0);
    }

    // In other words, the only acceptable number of voltage sensors is three.
    // set the parameters
    CLARKE_setScaleFactors(handle, alpha_sf, beta_sf);
    CLARKE_setNumSensors(handle, numVoltageSensors);

    return;
} // end of setupClarke_V() function

void init_can_references()
{
    can_0x240_motor1_feedback_data.position = 0;
    can_0x240_motor1_feedback_data.velocity = 0;
    can_0x240_motor1_feedback_data.effort = 0;

    can_0x230_motor2_feedback_data.position = 0;
    can_0x230_motor2_feedback_data.velocity = 0;
    can_0x230_motor2_feedback_data.effort = 0;

    // Initialize flags:
    can_0x100_mode_data = (can_0x100_mode_t){ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    gMotorVars[HAL_MTR1].Flag_Run_Identify = &can_0x100_mode_data.enable_motor_1;
    gMotorVars[HAL_MTR1].Flag_enableAlignment = &can_0x100_mode_data.elec_motor_1;
    gMotorVars[HAL_MTR1].Flag_enableMechAlignment = &can_0x100_mode_data.mech_motor_1;
    gMotorVars[HAL_MTR1].Flag_enableOffsetcalc = &can_0x100_mode_data.adc_motor_1;

    gMotorVars[HAL_MTR2].Flag_Run_Identify = &can_0x100_mode_data.enable_motor_2;
    gMotorVars[HAL_MTR2].Flag_enableAlignment = &can_0x100_mode_data.elec_motor_2;
    gMotorVars[HAL_MTR2].Flag_enableMechAlignment = &can_0x100_mode_data.mech_motor_2;
    gMotorVars[HAL_MTR2].Flag_enableOffsetcalc = &can_0x100_mode_data.adc_motor_2;

    gMotorVars[HAL_MTR1].Flag_position_control = &can_0x100_mode_data.position_control;
    gMotorVars[HAL_MTR1].Flag_cartesian_control = &can_0x100_mode_data.cartesian_control;
    gMotorVars[HAL_MTR1].Flag_current_control = &can_0x100_mode_data.current_control;
    gMotorVars[HAL_MTR2].Flag_position_control = &can_0x100_mode_data.position_control;
    gMotorVars[HAL_MTR2].Flag_cartesian_control = &can_0x100_mode_data.cartesian_control;
    gMotorVars[HAL_MTR2].Flag_current_control = &can_0x100_mode_data.current_control;
}

void send_status_msg()
{
    pack_message(0x100, &msg_data);
    send_msg(0x000, msg_data, DLC_2);
}

void send_msg(uint32_t msg_id, uint64_t data, uint32_t msg_len)
{
    uint32_t dataL = data & 0xffffffff;
    uint32_t dataH = (data >> 32) & 0xffffffff;
    FIFO_PUSH_ID(&gECAN_txFIFO_ID, msg_id | ECANIDS, msg_len,
        reverse_byte_order(dataL), reverse_byte_order(dataH));
}

void mdrive_init()
{
    // initialize the Hardware Abstraction Layer  (HAL)
    // halHandle will be used throughout the code to interface with the HAL
    // (set parameters, get and set functions, etc) halHandle is required since
    // this is how all objects are interfaced, and it allows interface with
    // multiple objects by simply passing a different handle. The use of
    // handles is explained in this document:
    // C:/ti/motorware/motorware_1_01_00_1x/docs/motorware_coding_standards.pdf
    halHandle = HAL_init(&hal, sizeof(hal));

    // initialize the user parameters
    // This function initializes all values of structure gUserParams with
    // values defined in user.h. The values in gUserParams will be then used by
    // the hardware abstraction layer (HAL) to configure peripherals such as
    // PWM, ADC, interrupts, etc.
    USER_setParamsMtr1(&gUserParams[HAL_MTR1]);
    USER_setParamsMtr2(&gUserParams[HAL_MTR2]);

    // set the hardware abstraction layer parameters
    // This function initializes all peripherals through a Hardware Abstraction
    // Layer (HAL). It uses all values stored in gUserParams.
    HAL_setParams(halHandle, &gUserParams[HAL_MTR1]);

    {
        kinematicsHandle = KINEMATICS_init(&kinematics, sizeof(kinematics));
        KINEMATICS_set_lengths(kinematicsHandle, _IQ(1.0), _IQ(1.0), _IQ(1.0),
            _IQ(1.0), _IQ(0.0));

        uint_least8_t mtrNum;

        for (mtrNum = HAL_MTR1; mtrNum <= HAL_MTR2; mtrNum++) {

            // initialize the individual motor hal files
            halHandleMtr[mtrNum] = HAL_init_mtr(
                &halMtr[mtrNum], sizeof(halMtr[mtrNum]), (HAL_MtrSelect_e)mtrNum);

            // Setup each motor board to its specific setting
            HAL_setParamsMtr(halHandleMtr[mtrNum], halHandle, &gUserParams[mtrNum]);

            // These function calls are used to initialize the estimator with ROM function calls. 
            // It needs the specific address where the controller
            // object is declared by the ROM code.

            // initialize the Clarke modules
            // Clarke handle initialization for current signals
            clarkeHandle_I[mtrNum] = CLARKE_init(&clarke_I[mtrNum], sizeof(clarke_I[mtrNum]));

            // Clarke handle initialization for voltage signals
            clarkeHandle_V[mtrNum] = CLARKE_init(&clarke_V[mtrNum], sizeof(clarke_V[mtrNum]));

            // Park handle initialization for current signals
            parkHandle[mtrNum] = PARK_init(&park[mtrNum], sizeof(park[mtrNum]));

            // compute scaling factors for flux and torque calculations
            gFlux_pu_to_Wb_sf[mtrNum] = USER_computeFlux_pu_to_Wb_sf(&gUserParams[mtrNum]);

            gFlux_pu_to_VpHz_sf[mtrNum] = USER_computeFlux_pu_to_VpHz_sf(&gUserParams[mtrNum]);

            gTorque_Ls_Id_Iq_pu_to_Nm_sf[mtrNum] = USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf(&gUserParams[mtrNum]);

            gTorque_Flux_Iq_pu_to_Nm_sf[mtrNum] = USER_computeTorque_Flux_Iq_pu_to_Nm_sf(&gUserParams[mtrNum]);

            gSpeed_krpm_to_pu_sf[mtrNum] = _IQ((float_t)gUserParams[mtrNum].motor_numPolePairs * 1000.0 / (gUserParams[mtrNum].iqFullScaleFreq_Hz * 60.0));

            gSpeed_pu_to_krpm_sf[mtrNum] = _IQ((gUserParams[mtrNum].iqFullScaleFreq_Hz * 60.0) / ((float_t)gUserParams[mtrNum].motor_numPolePairs * 1000.0));

            gSpeed_hz_to_krpm_sf[mtrNum] = _IQ(60.0 / (float_t)gUserParams[mtrNum].motor_numPolePairs / 1000.0);

            gCurrent_A_to_pu_sf[mtrNum] = _IQ(1.0 / gUserParams[mtrNum].iqFullScaleCurrent_A);

            // set the number of current sensors
            setupClarke_I(clarkeHandle_I[mtrNum],
                gUserParams[mtrNum].numCurrentSensors);

            // set the number of voltage sensors
            setupClarke_V(clarkeHandle_V[mtrNum],
                gUserParams[mtrNum].numVoltageSensors);

            // initialize the PID controllers
            pidSetup((HAL_MtrSelect_e)mtrNum);

            // initialize the inverse Park module
            iparkHandle[mtrNum] = IPARK_init(&ipark[mtrNum], sizeof(ipark[mtrNum]));

            // initialize the space vector generator module
            svgenHandle[mtrNum] = SVGEN_init(&svgen[mtrNum], sizeof(svgen[mtrNum]));

            // initialize and configure offsets using filters
            {
                uint16_t cnt = 0;
                _iq b0 = _IQ(gUserParams[mtrNum].offsetPole_rps / (float_t)gUserParams[mtrNum].ctrlFreq_Hz);
                _iq a1 = (b0 - _IQ(1.0));
                _iq b1 = _IQ(0.0);

                for (cnt = 0; cnt < 6; cnt++) {
                    filterHandle[mtrNum][cnt] = FILTER_FO_init(&filter[mtrNum][cnt], sizeof(filter[mtrNum][0]));
                    FILTER_FO_setDenCoeffs(filterHandle[mtrNum][cnt], a1);
                    FILTER_FO_setNumCoeffs(filterHandle[mtrNum][cnt], b0, b1);
                    FILTER_FO_setInitialConditions(filterHandle[mtrNum][cnt], _IQ(0.0),
                        _IQ(0.0));
                }

                *(gMotorVars[mtrNum].Flag_enableOffsetcalc) = 0;
            }

            // setup faults
            HAL_setupFaults(halHandleMtr[mtrNum]);

        } // End of for loop
    }

    // set the pre-determined current and voltage feedback offset values
    gOffsets_I_pu[HAL_MTR1].value[0] = _IQ(I_A_offset);
    gOffsets_I_pu[HAL_MTR1].value[1] = _IQ(I_B_offset);
    gOffsets_I_pu[HAL_MTR1].value[2] = _IQ(I_C_offset);
    gOffsets_V_pu[HAL_MTR1].value[0] = _IQ(V_A_offset);
    gOffsets_V_pu[HAL_MTR1].value[1] = _IQ(V_B_offset);
    gOffsets_V_pu[HAL_MTR1].value[2] = _IQ(V_C_offset);

    gOffsets_I_pu[HAL_MTR2].value[0] = _IQ(I_A_offset_2);
    gOffsets_I_pu[HAL_MTR2].value[1] = _IQ(I_B_offset_2);
    gOffsets_I_pu[HAL_MTR2].value[2] = _IQ(I_C_offset_2);
    gOffsets_V_pu[HAL_MTR2].value[0] = _IQ(V_A_offset_2);
    gOffsets_V_pu[HAL_MTR2].value[1] = _IQ(V_B_offset_2);
    gOffsets_V_pu[HAL_MTR2].value[2] = _IQ(V_C_offset_2);

    HAL_disableGlobalInts(halHandle);
    // HAL_disableDebugInt(halHandle);
    // EEPROM_init();
    // PERSIST_loadAll();

    // initialize the interrupt vector table
    HAL_initIntVectorTable(halHandle);
    // enable the ADC interrupts
    HAL_enableAdcInts(halHandle);
    // enable global interrupts
    HAL_enableGlobalInts(halHandle);
    // enable debug interrupts
    HAL_enableDebugInt(halHandle);

    // disable the PWM
    HAL_disablePwm(halHandleMtr[HAL_MTR1]);
    HAL_disablePwm(halHandleMtr[HAL_MTR2]);

    // enable the system by default
    gMotorVars[HAL_MTR1].Flag_enableSys = true;

#ifdef DRV8301_SPI
    // turn on the DRV8301 if present
    HAL_setupDRV8301SpiA(halHandle);
    HAL_setupDRV8301SpiB(halHandle);
    HAL_enableDrv(halHandleMtr[HAL_MTR1]);
    HAL_enableDrv(halHandleMtr[HAL_MTR2]);
    // initialize the DRV8301 interface
    HAL_setupDrvSpi(halHandleMtr[HAL_MTR1], &gDrvSpi8301Vars[HAL_MTR1]);
    HAL_setupDrvSpi(halHandleMtr[HAL_MTR2], &gDrvSpi8301Vars[HAL_MTR2]);
    HAL_setupSpiA(halHandle);
    HAL_setupSpiB(halHandle);
#endif
    init_can_references();
    ECAN_initMailboxUse(&gECAN_Mailbox, MailBox15, MailBox0, MailBox31,
        MailBox16);
    ECAN_SelfTest(halHandle->ecanaHandle, Normal_mode);
    FIFO_FLUSH(gECAN_rxFIFO_ID);
    FIFO_FLUSH(gECAN_txFIFO_ID);
}

void cartesianControlComp()
{
    static _iq xPos = _IQ(0.0);
    static _iq yPos = _IQ(0.0);

    // get cartesian position.
    theta1 = _IQmpy(_IQ(6.28), HAL_getEncoderMechanicalAngle(halHandleMtr[HAL_MTR1]));
    theta2 = _IQmpy(_IQ(6.28), HAL_getEncoderMechanicalAngle(halHandleMtr[HAL_MTR2]));
    KINEMATICS_compute_jacobian(kinematicsHandle, theta1, theta2);
    jacobian[0][0] = kinematicsHandle->Jacobian[0][0];
    jacobian[1][0] = kinematicsHandle->Jacobian[1][0];
    jacobian[0][1] = kinematicsHandle->Jacobian[0][1];
    jacobian[1][1] = kinematicsHandle->Jacobian[1][1];
    KINEMATICS_compute_fk(kinematicsHandle, _IQ(0.5), _IQ(2));
    _iq xPosNew = KINEMATICS_get_x(kinematicsHandle);
    _iq yPosNew = KINEMATICS_get_y(kinematicsHandle);
    _iq speedX = xPosNew - xPos;
    _iq speedY = yPosNew - yPos;
    xPos = xPosNew;
    yPos = yPosNew;
    _iq errorX = xPos - _IQ(gCommand[0]);
    _iq errorY = yPos - _IQ(gCommand[1]);

    // PD controller in cartesian space to get Fx, Fy
    _iq Fx = _IQmpy(gCartesianGains[0][0], errorX) + _IQmpy(gCartesianGains[0][1], speedX);
    _iq Fy = _IQmpy(gCartesianGains[1][0], errorY) + _IQmpy(gCartesianGains[1][1], speedY);

    // Multiply Fx, Fy by jacobian transpose to get Tau1 Tau2 (motor commands)
    _iq motor1Command = _IQmpy(jacobian[0][0], Fx) + _IQmpy(jacobian[1][0], Fy);
    _iq motor2Command = _IQmpy(jacobian[0][1], Fx) + _IQmpy(jacobian[1][1], Fy);

    // Set Q-axis current to command
    gIdq_ref_pu[HAL_MTR1].value[1] = _IQdiv(motor1Command, _IQ(USER_IQ_FULL_SCALE_CURRENT_A));
    gIdq_ref_pu[HAL_MTR2].value[1] = _IQdiv(motor2Command, _IQ(USER_IQ_FULL_SCALE_CURRENT_A));

    // Set D-axis current to 0
    gIdq_ref_pu[HAL_MTR1].value[0] = _IQ(0.0);
    gIdq_ref_pu[HAL_MTR2].value[0] = _IQ(0.0);
}
