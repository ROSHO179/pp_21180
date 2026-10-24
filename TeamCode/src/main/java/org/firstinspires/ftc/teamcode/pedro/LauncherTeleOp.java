package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.procedures.robot.Launcher;

@TeleOp(name = "Launcher TeleOp")
public class LauncherTeleOp extends LinearOpMode {
    public static double AT_SPEED_TOLERANCE = 50.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.create(hardwareMap);
        Drivetrain drivetrain = follower != null ? follower.drivetrain : null;
        Launcher launcher = new Launcher(hardwareMap);
        ElapsedTime timer = new ElapsedTime();

        if (drivetrain == null) {
            telemetry.addLine("Constants.create() returned null: drivetrain disabled, launcher only.");
            telemetry.update();
        }

        waitForStart();
        timer.reset();
        double lastTime = timer.seconds();

        while (opModeIsActive()) {
            double now = timer.seconds();
            double dt = now - lastTime;
            if (dt <= 0) dt = 1e-6;
            lastTime = now;

            if (drivetrain != null) {
                drivetrain.drive(new DrivePowers(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x), true);
            }

            if (gamepad1.right_trigger > 0.5) {
                launcher.spoolUp();
            } else {
                launcher.spoolDown();
            }

            if (gamepad1.a) {
                launcher.openSwitch();
            } else if (gamepad1.b) {
                launcher.closeSwitch();
            }

            launcher.update(dt);

            telemetry.addData("Launcher target (tps)", launcher.getTargetVelocity());
            telemetry.addData("Launcher commanded (tps)", launcher.getCommandedVelocity());
            telemetry.addData("Launcher actual (tps)", launcher.getMeasuredVelocity());
            telemetry.addData("Launcher L / R (tps)", "%.0f / %.0f", launcher.getLeftVelocity(), launcher.getRightVelocity());
            telemetry.addData("At speed", launcher.isAtSpeed(AT_SPEED_TOLERANCE));
            telemetry.addData("dt (ms)", dt * 1000.0);
            telemetry.update();
        }
    }
}
