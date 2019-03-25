package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.configuration.annotations.AnalogSensorType;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="FTC 2019 TeleOP", group="Iterative Opmode")

public class FTC_2019_Tele extends OpMode
{
    //Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor_front_left;
    private DcMotor motor_front_right;
    private DcMotor motor_back_left;
    private DcMotor motor_back_right;
    private DcMotor motor_arm_right;
    private DcMotor motor_arm_left;
    private DcMotor motor_extension;
    private DcMotor motor_latching;
    private CRServo servo_intake;
    private AnalogInput pot_arm;
    double motor_arm_right_power = 0;
    double motor_arm_left_power = 0;
    double turn_control = 2.5;
    boolean arm_down = true;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        //Initialize the hardware variables
        motor_front_left  = hardwareMap.get(DcMotor.class, "motor front left");
        motor_front_right  = hardwareMap.get(DcMotor.class, "motor front right");
        motor_back_left  = hardwareMap.get(DcMotor.class, "motor back left");
        motor_back_right  = hardwareMap.get(DcMotor.class, "motor back right");
        motor_arm_right  = hardwareMap.get(DcMotor.class, "motor arm right");
        motor_arm_left  = hardwareMap.get(DcMotor.class, "motor arm left");
        motor_latching  = hardwareMap.get(DcMotor.class, "motor latching");
        motor_extension  = hardwareMap.get(DcMotor.class, "motor extension");
        servo_intake = hardwareMap.get(CRServo.class, "servo intake");
        pot_arm = hardwareMap.analogInput.get("pot arm");
        motor_arm_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_arm_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_arm_left.setDirection(DcMotor.Direction.REVERSE);
        motor_latching.setDirection(DcMotor.Direction.REVERSE);
        
        //Initialize encoders
        motor_arm_right.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_arm_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_arm_left.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_arm_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        //set brake behaviour
        motor_latching.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
    }
    
    // Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
    }

    ///Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
            
            //calculate power for drive motors with joysticks
            double motor_front_left_power  = (gamepad1.left_stick_y - gamepad1.left_stick_x)/1.8;
            double motor_front_right_power  = (-gamepad1.left_stick_y - gamepad1.left_stick_x)/1.8;
            double motor_back_left_power  = (+gamepad1.left_stick_y + gamepad1.left_stick_x)/1.8;
            double motor_back_right_power  = (-gamepad1.left_stick_y + gamepad1.left_stick_x)/1.8;
            
            motor_front_left_power  += (gamepad1.right_stick_x)/turn_control;
            motor_front_right_power  += (gamepad1.right_stick_x)/turn_control;
            motor_back_left_power  += (gamepad1.right_stick_x)/turn_control;
            motor_back_right_power  += (gamepad1.right_stick_x)/turn_control;
            
            //latcher
            double motor_latching_power = (gamepad1.left_trigger - gamepad1.right_trigger);
            
            //linear extension
            double motor_extension_power = (gamepad2.right_trigger - gamepad2.left_trigger);
            
            //arm in grabbing position
            if(gamepad2.dpad_down == true) 
            {
                arm_down = true;
            }
            
            //arm in depositing position
            if(gamepad2.dpad_up == true) {
                arm_down = false;
                motor_arm_left.setTargetPosition(1810);
                motor_arm_right.setTargetPosition(1810);
                motor_arm_left_power = 0.2;
                motor_arm_right_power = 0.2;
            }
            
            //arm in upright position
            if(gamepad2.dpad_left == true) {
                arm_down = false;
                motor_arm_left.setTargetPosition(1250);
                motor_arm_right.setTargetPosition(1250);
                if(motor_arm_right.getCurrentPosition() < 1000)
                {
                    motor_arm_left_power = 0.35;
                    motor_arm_right_power = 0.35;
                } else {
                    motor_arm_left_power = 0.15;
                    motor_arm_right_power = 0.15;
                }
            }
            
            //reset arm encoders
            if(gamepad2.a == true) {
                motor_arm_left.setPower(0);
                motor_arm_right.setPower(0);
                motor_arm_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motor_arm_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motor_arm_left.setMode(DcMotor.RunMode.RESET_ENCODERS);
                motor_arm_right.setMode(DcMotor.RunMode.RESET_ENCODERS);
                motor_arm_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_arm_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } 
            
            //reset arm using potentiometer (only to be used in cases of desync of encoders or if robot is flipped)
            if(gamepad2.x == true) {
                if((Math.round(pot_arm.getVoltage() / 0.033 * 3.6 * 10d) / 10d) > 80) {
                    arm_down = false;
                    motor_arm_left.setTargetPosition(-3000);
                    motor_arm_right.setTargetPosition(-3000);
                    motor_arm_right_power = 0.3;
                    motor_arm_left_power = 0.3;
                } else {
                    arm_down = true;
                }
            }

            //checks if arm is already in grabbing position
            if((Math.round(pot_arm.getVoltage() / 0.033 * 3.6 * 10d) / 10d) > 80) {
                arm_down = true;
            }

            //arm in grabbing position
            if(arm_down == true)
            {
                turn_control = 3.5;
                if(motor_arm_right.getCurrentPosition() < 1000) {
                    motor_arm_left.setTargetPosition(0);
                    motor_arm_right.setTargetPosition(0);
                    motor_arm_left_power =0;
                    motor_arm_right_power =0;
                    motor_arm_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    motor_arm_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                } else {
                    motor_arm_left.setTargetPosition(900);
                    motor_arm_right.setTargetPosition(900);
                    motor_arm_left_power = 0.15;
                    motor_arm_right_power = 0.15;
                }
            } else {
                turn_control = 2.5;
            }
            // Send calculated power to motors
            motor_front_left.setPower(motor_front_left_power);
            motor_front_right.setPower(motor_front_right_power);
            motor_back_left.setPower(motor_back_left_power);
            motor_back_right.setPower(motor_back_right_power);
            motor_latching.setPower(motor_latching_power);
            motor_extension.setPower(motor_extension_power);
            motor_arm_right.setPower(motor_arm_right_power);
            motor_arm_left.setPower(motor_arm_left_power);
        
            //intake control
            if(gamepad2.right_bumper == true || gamepad1.right_bumper == true) {
                servo_intake.setPower(1);
            } else {
                if(gamepad2.left_bumper == true || gamepad1.left_bumper == true) {
                servo_intake.setPower(-1);
                } else {
                    servo_intake.setPower(0);
                }
            }
            
        
            // Show the elapsed game time, encoder and potentiometer value
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Potentiometer",Double.valueOf(Math.round(pot_arm.getVoltage() / 0.033 * 3.6 * 10d) / 10d).toString() + "°");
            telemetry.addData("Arm",Integer.valueOf(motor_arm_left.getCurrentPosition()).toString());
            telemetry.addData("Latcher",Integer.valueOf(motor_latching.getCurrentPosition()).toString());
            telemetry.update();
    }
    
    @Override
    public void stop() {
    }
}
