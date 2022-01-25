package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "robot2drive2 (Blocks to Java)")
public class elapsedtimetest extends LinearOpMode {

  private ElapsedTime     runtime = new ElapsedTime();

  private DcMotor right_rear_motor;
  private DcMotor right_front_motor;
  private DcMotor left_rear_motor;
  private DcMotor left_front_motor;
  private DcMotor strafe_motor;

  private CRServo right_intake;
  private CRServo left_intake;

  //power to left/right side of drivetrain
  double L;
  double R;

  //drive constants, to control power
  double masterSpeed = 0.4;
  double turnSpeed = 0.3;

  //values for strafe control
  double strafe_increment = 0.001;
  double strafe_pow = 0;
  double max_strafe_pow = 0.4;

  @Override
  public void runOpMode() {
    //hardware configuration, naming all motors/servos and configuring direction/behaviour

    //motor config
    right_rear_motor = hardwareMap.get(DcMotor.class, "right_rear_motor");
    right_front_motor = hardwareMap.get(DcMotor.class, "right_front_motor");
    left_rear_motor = hardwareMap.get(DcMotor.class, "left_rear_motor");
    left_front_motor = hardwareMap.get(DcMotor.class, "left_front_motor");
    strafe_motor = hardwareMap.get(DcMotor.class, "strafe_motor");

    //servo config
    right_intake = hardwareMap.get(CRServo.class, "right_intake");
    left_intake = hardwareMap.get(CRServo.class, "left_intake");

    //direction fixing (so all motors drive in the same direction)
    right_rear_motor.setDirection(DcMotorSimple.Direction.REVERSE);
    right_front_motor.setDirection(DcMotorSimple.Direction.REVERSE);
    left_rear_motor.setDirection(DcMotorSimple.Direction.REVERSE);

    right_intake.setDirection(DcMotorSimple.Direction.REVERSE);

    //prevent robot from rolling when power is cut
    right_rear_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    right_front_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    left_rear_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    left_front_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    //opMode loop, this is what actually runs when you press start.
    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        //main drive loop, methods here are called repeatedly while active
        Drive();
        Intake();
        Telemetry();
      }
    }
  }

  // drive method, handles all driving-related behavior.
  private void Drive() {
    //check if speed boost is enabled
    if (gamepad2.right_trigger > 0.5) {
        masterSpeed = 0.8;
    } else {
        masterSpeed = 0.4;
    }
    //right stick, used for driving straight
    if (-0.2 < gamepad2.right_stick_x && gamepad2.right_stick_x < 0.2) {
      L = masterSpeed * gamepad2.left_stick_y;
      R = masterSpeed * gamepad2.left_stick_y;
    }
    //left stick, used for turning
    if (-0.2 < gamepad2.left_stick_y && gamepad2.left_stick_y < 0.2) {
      L = turnSpeed * -gamepad2.right_stick_x;
      R = turnSpeed * gamepad2.right_stick_x;
    }
    //strafing
    if (gamepad2.left_bumper) {
      Strafe(1);
    } else if (gamepad2.right_bumper) {
      Strafe(-1);
    } else {
      strafe_pow = 0;
    }

    //assigning power values to motors
    right_rear_motor.setPower(R);
    right_front_motor.setPower(R);
    left_rear_motor.setPower(L);
    left_front_motor.setPower(L);
    strafe_motor.setPower(strafe_pow);
  }

  //strafe code, used in drive method.
  private void Strafe(int goal) {
    if (goal == 1 && strafe_pow < max_strafe_pow) {
      //if strafe power isn't at max, increment
      strafe_pow += (strafe_increment * goal);
    } 
    if (goal == -1 && strafe_pow > (max_strafe_pow * -1)) {
      //same for other direction
      strafe_pow += (strafe_increment * goal);
    }
  }

  //intake code, to take in cargo.
  private void Intake() {
    if (gamepad2.x) {
      Intake_power(1);
    } else if(gamepad2.y) {
      Intake_power(-1);
    } else {
      Intake_power(0);
    }
  }

  //set intake power
  private void Intake_power(int d) {
    right_intake.setPower(d);
    left_intake.setPower(d);
  }

  //telemetry updates, to see info live, while robot is active
  private void Telemetry() {
    telemetry.addData("Strafe", strafe_pow);
    telemetry.addData("intakeleft", left_intake);
    telemetry.addData("intakeright", right_intake);
    telemetry.addData("Left", L);
    telemetry.addData("Right", R);
    telemetry.addData("Elapsed Time", runtime.seconds());
    telemetry.addData("Elapsed Time -400ms", runtime.seconds() - 0.4);
    telemetry.update();
  }
}
