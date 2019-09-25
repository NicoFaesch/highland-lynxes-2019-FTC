package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


//ObjectDetection
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



@Autonomous(name="FTC_2019_Auto_Depot", group="Iterative Opmode")
    

public class FTC_2019_Auto_Depot extends LinearOpMode
{
    // Object Detection 
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "Ae64HsX/////AAABmTefiXoWzkfqpCBol+bLoS2BpnqvqgQUmND/o15P7uK2FW/KnaiKs+iwyWmkUTxffjLZfvkzkrNCoUFQD/h0l6CgroHLOK3wqSaUxT1u8wCaLRcmMuBRAadEJSHPddJ9kKOCnCU6yy2NO+7nnB5oIXBwmMStHJCEmSV4KsR3rSKhv18NUK1R0jKA9VRg7lQzPncPfiP6Csjy1Cmzxkf4Gpd7MfdZRl/fIQp/sReOSWRBNstQ9GHyYyIR65o6HPl3gjVRAaJbYap5WMmmsRumQju0/jZOHtXviV3Z3+5/Sw9a2SsAhoaLaefx9I5SDka9dUSgD7hkfD5XxTFuwXYRNRmC6zAV+5ki2i6fKyufdMWe";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    //Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor_front_left;
    private DcMotor motor_front_right;
    private DcMotor motor_back_left;
    private DcMotor motor_back_right;
    private DcMotor motor_arm_left;
    private DcMotor motor_arm_right;
    private DcMotor motor_latching;
    private Servo servo_marker;
    
    //declare variables for motor speed
    private double motor_front_left_power = 0;
    private double motor_front_right_power = 0;
    private double motor_back_left_power = 0;
    private double motor_back_right_power = 0;
    
    //speed variable for movement
    private double speed = 0.5;
    
    //declare variable for orientation
    private double sampling_gold = 0;
    private boolean gold = false;
    private double kP = 0.00025;
    private double kD = 0.001;
    private double error_fl = 0;
    private double error_fr = 0;
    private double error_bl = 0;
    private double error_br = 0;
    private double last_error_fl = 0;
    private double last_error_fr = 0;
    private double last_error_bl = 0;
    private double last_error_br = 0;
    private double averagemotor = 0;
        
    //directions for 'drive' function
    private enum Direction {
        FORWARD, 
        BACKWARD,
        LEFT,
        RIGHT,
        TURN_LEFT,
        TURN_RIGHT
    }
    
    //function that allows to drive the tank
    private void drive(Direction direction, int angle) {
        switch (direction) {
            case FORWARD:
                motor_front_left_power  =  speed;
                motor_front_right_power = -speed;
                motor_back_left_power   =  speed;
                motor_back_right_power  = -speed;
                break;
            case BACKWARD:
                motor_front_left_power  = -speed;
                motor_front_right_power =  speed;
                motor_back_left_power   = -speed;
                motor_back_right_power  =  speed;
                break;
            case LEFT:
                motor_front_left_power  = -speed;
                motor_front_right_power = -speed;
                motor_back_left_power   =  speed;
                motor_back_right_power  =  speed;
                break;
            case RIGHT:
                motor_front_left_power  =  speed;
                motor_front_right_power =  speed;
                motor_back_left_power   = -speed;
                motor_back_right_power  = -speed;
                break;
            case TURN_LEFT:
                speed = 0.4;
                motor_front_left_power  = -speed;
                motor_front_right_power = -speed;
                motor_back_left_power   = -speed;
                motor_back_right_power  = -speed;
                break;
            case TURN_RIGHT:
                speed = 0.4;
                motor_front_left_power  =  speed;
                motor_front_right_power =  speed;
                motor_back_left_power   =  speed;
                motor_back_right_power  =  speed;
                break;
            default:
                break;
        }
        
        // reset encoders
        motor_front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        motor_front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        motor_back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        motor_back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // wait for end of execution
        int isRunning = 0b1111;
        while (opModeIsActive() && (isRunning > 0)) {
            // proportial 
            averagemotor = (Math.abs(motor_back_left.getCurrentPosition()) + Math.abs(motor_back_right.getCurrentPosition()) + Math.abs(motor_front_left.getCurrentPosition()) + Math.abs(motor_front_right.getCurrentPosition())) / 4;
    
            // set Power
            if (Math.abs(motor_front_left.getCurrentPosition()) > Math.abs(angle)) {
                isRunning &= 0b1110;
                motor_front_left_power = 0;
            } else {
                error_fl = averagemotor - (Math.abs(motor_front_left.getCurrentPosition()));
                motor_front_left_power += (error_fl * kP + (error_fl - last_error_fl) * kD) * (motor_front_left_power / Math.abs(motor_front_left_power));
                last_error_fl = error_fl;
            }
            if (Math.abs(motor_front_right.getCurrentPosition()) > Math.abs(angle)) {
                isRunning &= 0b1101;
                motor_front_right_power = 0;
            } else {
                error_fr = averagemotor - (Math.abs(motor_front_right.getCurrentPosition()));
                motor_front_right_power += (error_fr * kP + (error_fr - last_error_fr) * kD) * (motor_front_right_power / Math.abs(motor_front_right_power));
                last_error_fr = error_fr;
            }
            if (Math.abs(motor_back_left.getCurrentPosition()) > Math.abs(angle)) {
                isRunning &= 0b1011;
                motor_back_left_power = 0;
            } else {
                error_bl = averagemotor - (Math.abs(motor_back_left.getCurrentPosition()));
                motor_back_left_power += (error_bl * kP + (error_bl - last_error_bl) * kD)* (motor_back_left_power / Math.abs(motor_back_left_power));
                last_error_bl = error_bl;
            }
            if (Math.abs(motor_back_right.getCurrentPosition()) > Math.abs(angle)) {
                isRunning &= 0b0111;
                motor_back_right_power = 0;
            } else {
                error_br = averagemotor - (Math.abs(motor_back_right.getCurrentPosition()));
                motor_back_right_power += (error_br * kP + (error_br - last_error_br) * kD)* (motor_back_right_power / Math.abs(motor_back_right_power));
                last_error_br = error_br;
            }
            
            // apply power
            motor_front_left.setPower(motor_front_left_power);
            motor_front_right.setPower(motor_front_right_power);
            motor_back_left.setPower(motor_back_left_power);
            motor_back_right.setPower(motor_back_right_power);
        }
        
        //sleeps
        speed = 0.5;
        sleep(150);
    }
    
    
    //checks for gold
    private boolean isgold() {
        boolean detected = false;
        double counterStart = getRuntime();
        gold = false;
        tfod.activate();
        if (opModeIsActive()) {
            if (tfod != null) {
                while (opModeIsActive() && detected == false) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());
                      if (updatedRecognitions.size() == 1) {
                        for (Recognition recognition : updatedRecognitions) {
                          if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            telemetry.addData("Mineral:", "gold");
                            gold =  true;
                            detected = true;
                          } else  {
                            telemetry.addData("Mineral:", "silver");
                            gold =  false;
                            detected = true;
                            }
                        }
                        telemetry.update();
                      } else {
                          if(getRuntime() >= counterStart + 1.5) {
                            telemetry.addData("Mineral:", "silver(guess)");
                            gold =  false;
                            detected = true;
                          }
                      }
                      
                    }
                }
            }
        }
        return gold;
    }
    
       /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
        "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.3;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    @Override
    public void runOpMode() {
        //INIT
        motor_front_left = hardwareMap.get(DcMotor.class, "motor front left");
        motor_front_right = hardwareMap.get(DcMotor.class, "motor front right");
        motor_back_left = hardwareMap.get(DcMotor.class, "motor back left");
        motor_back_right = hardwareMap.get(DcMotor.class, "motor back right");
        motor_arm_left = hardwareMap.get(DcMotor.class, "motor arm left");
        motor_arm_right = hardwareMap.get(DcMotor.class, "motor arm right");
        motor_latching = hardwareMap.get(DcMotor.class, "motor latching");
        servo_marker = hardwareMap.get(Servo.class, "servo marker");
        motor_latching.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_arm_left.setDirection(DcMotor.Direction.REVERSE);
        motor_latching.setDirection(DcMotor.Direction.REVERSE);
        
        // Initialize servos
        servo_marker.setPosition(0.82);
        
        //reset encoder
        motor_arm_left.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_arm_right.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_arm_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_arm_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        if(opModeIsActive()) {
            if (tfod != null) {
                tfod.activate();
            }
        }
        //brake latcher
        motor_latching.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_latching.setTargetPosition(motor_latching.getCurrentPosition());
        motor_latching.setPower(0);

        //initialize vuforia and TensorFlow
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //START
        /* waitForStart(); replaced by the following lines due
         * motorola phones disconnecting :
         */
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start");
            telemetry.update();
        }
        // end motorola-specific fix
        
        runtime.reset();
        telemetry.update();
        telemetry.addData("Lauf", "?");
        telemetry.update();
        int SamplingPosition = -1;
        motor_latching.setTargetPosition(motor_latching.getCurrentPosition() + 4600);
        motor_latching.setPower(1);
        // original
        sleep(9200);
        
        /* new
        for (int i = 0; i < 93; i++) {
            telemetry.update();
            sleep(100);
        }
        */
        motor_latching.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_latching.setTargetPosition(motor_latching.getCurrentPosition());
        motor_latching.setPower(0);
        drive(Direction.FORWARD, 20);
        drive(Direction.RIGHT, 80);
        drive(Direction.BACKWARD, 150);
        drive(Direction.LEFT, 80);
        if(isgold()){
            //drive(Direction.RIGHT, 80);
            drive(Direction.BACKWARD , 900);
            telemetry.addData("gold is in the", "center");
            telemetry.update();
            SamplingPosition = 1;
        }else{
            drive(Direction.TURN_RIGHT, 200);
            if(isgold()){
                drive(Direction.BACKWARD,990);
                telemetry.addData("gold is in the", "right");
                telemetry.update();
                SamplingPosition = 0;
            }else{
                drive(Direction.TURN_LEFT, 510);
                sleep(300);
                drive(Direction.BACKWARD,950);
                telemetry.addData("gold is in the", "left");
                telemetry.update();
                SamplingPosition = 2;
            }
        }
        if(tfod != null) {
            tfod.shutdown();
        }
        // drive to further position
        if(SamplingPosition == 0){
            drive(Direction.TURN_LEFT, 180);
            drive(Direction.BACKWARD, 300);
            drive(Direction.TURN_LEFT, 180);
            drive(Direction.BACKWARD, 1030);
            drive(Direction.TURN_RIGHT, 600);
            speed = 0.7;
            drive(Direction.FORWARD, 200);
        }else {
            if(SamplingPosition == 1) {
                drive(Direction.BACKWARD, 950);
                drive(Direction.TURN_RIGHT, 270);
                drive(Direction.FORWARD, 80);
                drive(Direction.RIGHT, 300);
            }else {
                drive(Direction.BACKWARD, 300);
                drive(Direction.TURN_RIGHT, 220);
                drive(Direction.BACKWARD, 300);
                drive(Direction.TURN_RIGHT, 220);
                drive(Direction.BACKWARD, 270);
            }
        }
        // drop teammarker
        servo_marker.setPosition(0.6);
        motor_front_left_power  = speed;
        motor_front_right_power = speed;
        motor_back_left_power   = -speed;
        motor_back_right_power  = -speed;
        motor_front_left.setPower(motor_front_left_power);
        motor_front_right.setPower(motor_front_right_power);
        motor_back_left.setPower(motor_back_left_power);
        motor_back_right.setPower(motor_back_right_power);
        sleep(1000);
        drive(Direction.LEFT, 100);
        //drive(Direction.TURN_RIGHT, 5);
        speed = 0.7;
        drive(Direction.FORWARD, 2580);
        sleep(1000);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
}
