package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Config //Disable if not using FTC Dashboard https://github.com/PinkToTheFuture/OpenCV_FreightFrenzy_2021-2022#opencv_freightfrenzy_2021-2022
@Autonomous(name= "OpenCV Test")
//@Disabled
public class OpenCVTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    /*
    Open CV Stuff
     */
    private OpenCvCamera phoneCam;
    private ContourPipelineGreen pipeline;

    private Telemetry telemetrydb;

    private double crThreshHigh = 150;
    private double crThreshLow = 120;
    private double cbThreshHigh = 255;
    private double cbThreshLow = 255;

    private int minRectangleArea = 2000;
    private double leftBarcodeRangeBoundary = 0.3; //i.e 30% of the way across the frame from the left
    private double rightBarcodeRangeBoundary = 0.6; //i.e 60% of the way across the frame from the left

    private double lowerRuntime = 0;
    private double upperRuntime = 0;

    // Pink Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 0.0, 0.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 120.0, 120.0);

    int x = 0;

    /*
    Mechanisms
     */
    // declare all of the servo and motor objects
    private DcMotor BackRight;
    private Servo budsterupanddown;
    private Servo ElliottispotatoClaw;
    private Servo LiftRight;
    private Servo LiftLeft;
    private DcMotor BackLeft;
    private DcMotor FrontLeft;
    private DcMotor FrontRight;

    // declare position variables
    double NewLiftPos;
    double LiftHeight;
    double FbFHeight;
    boolean LiftUpButtonIsPressed;
    boolean LiftDownButtonPressed;
    double MaxLiftHeight;
    double minLiftHeight;
    int NumLiftStops;
    double LiftLeftOffset;

    //BNO055IMU imu;                // Additional Gyro device
    //Orientation angles;

    private final static double ServoPosition = 0.5;
    private final static double ServoSpeed = 0.1;

    static final double DRIVE_SPEED = 0.45;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.40;     // Nominal half speed for better accuracy.

    //PID control constants
    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.025;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.007;     // Larger is more responsive, but also less stable

    @Override
    public void runOpMode() throws InterruptedException {


/*
OpenCV Stuff
 */

// OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //OpenCV Pipeline

        pipeline = new ContourPipelineGreen(0.2, 0.2, 0.2, 0.2);

        pipeline.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
        pipeline.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);

        phoneCam.setPipeline(pipeline);

        // Webcam Streaming
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        double rectangleArea = pipeline.getRectArea();
        double position = pipeline.getRectMidpointX();

        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        budsterupanddown = hardwareMap.get(Servo.class, "budsterupanddown");
        ElliottispotatoClaw = hardwareMap.get(Servo.class, "ElliottispotatoClaw");
        LiftRight = hardwareMap.get(Servo.class, "LiftRight");
        LiftLeft = hardwareMap.get(Servo.class, "LiftLeft");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");

        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        budsterupanddown.setDirection(Servo.Direction.REVERSE);
        ElliottispotatoClaw.setDirection(Servo.Direction.REVERSE);
        LiftRight.setDirection(Servo.Direction.REVERSE);
        FbFHeight = 0.3;

        // Lift Variables
        LiftHeight = 0.15;
        LiftLeftOffset = -0.022;
        MaxLiftHeight = 0.6;
        minLiftHeight = 0.12;
        NumLiftStops = 4;
        LiftDownButtonPressed = false;
        LiftUpButtonIsPressed = false;
        budsterupanddown.setPosition(0.5000000001);
        LiftLeft.setPosition(LiftLeftOffset + LiftHeight);
        LiftRight.setPosition(LiftHeight);
        NewLiftPos = LiftRight.getPosition();

        //Set motor modes
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Setting motor mode regarding encoders
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Mode", "working...");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        telemetry.addData("Mode", "ready");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
         */

        // Only if you are using ftcdashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetrydb = new MultipleTelemetry(telemetrydb, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(phoneCam, 10);
        telemetrydb.update();
        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            rectangleArea = pipeline.getRectArea();
            position = pipeline.getRectMidpointX();

            //sleep(1500);

            //sleep(1000000000);
        }
    }





}



