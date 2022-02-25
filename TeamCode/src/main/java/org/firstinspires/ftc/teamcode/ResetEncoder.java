package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Reset Encoder (Blocks to Java)")
public class ResetEncoder extends LinearOpMode {

  private ElapsedTime     runtime = new ElapsedTime();

  //initializing all motors and servos (making code recognize motors and assigning names)

  private DcMotor rightRearMotor;
  private DcMotor rightFrontMotor;
  private DcMotor leftRearMotor;
  private DcMotor leftFrontMotor;
  private DcMotor strafeMotor;
  private DcMotor chainMotor;

  private CRServo rightIntake;
  private CRServo leftIntake;
  private CRServo rightCarousel;
  private CRServo leftCarousel;

  private CRServo topIntake;

  //power to left/right side of drivetrain
  double leftDrivePower;
  double rightDrivePower;
  double strafePower;

  //drive constants, to control power



  @Override
  public void runOpMode() {
    //hardware configuration, naming all motors/servos and configuring direction/behaviour

    //motor config
    rightRearMotor = hardwareMap.get(DcMotor.class, "right_rear_motor");
    rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front_motor");
    leftRearMotor = hardwareMap.get(DcMotor.class, "left_rear_motor");
    leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front_motor");
    strafeMotor = hardwareMap.get(DcMotor.class, "strafe_motor");
    chainMotor = hardwareMap.get(DcMotor.class, "chain_motor");

    //servo config
    rightIntake = hardwareMap.get(CRServo.class, "right_intake");
    leftIntake = hardwareMap.get(CRServo.class, "left_intake");
    topIntake = hardwareMap.get(CRServo.class, "top_intake");

    rightCarousel = hardwareMap.get(CRServo.class, "right_carousel");
    leftCarousel = hardwareMap.get(CRServo.class, "left_carousel");

    //direction fixing (so all motors drive in the same direction)
    leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    topIntake.setDirection(DcMotorSimple.Direction.REVERSE);

    //prevent robot from rolling when power is cut
    rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    //opMode loop, this is what actually runs when you press start.
    waitForStart();
    if (opModeIsActive()) {
      //stuff here will be done once when "INIT" is pressed (driver hub)
      chainMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      while (opModeIsActive()) {
        //main drive loop, methods here are called repeatedly while active
        Telemetry();
      }
    }
  }



  //telemetry updates, to see info live while robot is active
  private void Telemetry() {
    telemetry.addLine("Bartholomew V2");
    telemetry.addData("Chain pow", chainMotor.getPower());
    telemetry.addData("Chain encoder", chainMotor.getCurrentPosition());


    telemetry.update();
  }
}