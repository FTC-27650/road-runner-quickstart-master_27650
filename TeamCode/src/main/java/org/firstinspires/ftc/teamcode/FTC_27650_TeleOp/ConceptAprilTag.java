/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.FTC_27650_TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * 本OpMode展示了AprilTag识别和位姿估计的基础知识，
 * 包括用于指定视觉参数的Java Builder结构。
 *
 * 有关AprilTags的介绍，请参见下面的FTC-DOCS链接：
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * 在本示例中，任何可见的标签ID都会被检测并显示，但只有包含在默认
 * "TagLibrary"中的标签才会显示其位置和方向信息。此默认标签库包含
 * 当前赛季的AprilTags和一小部分高编号范围的"测试标签"。
 *
 * 当检测到标签库中的AprilTag时，SDK会提供标签相对于相机的位置和方向。
 * 此信息在返回的"detection"的"ftcPose"成员中提供，并在下面链接的ftc-docs页面中进行了解释。
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * 要尝试使用AprilTags进行导航，请试用以下两个驱动示例：
 * RobotAutoDriveToAprilTagOmni和RobotAutoDriveToAprilTagTank
 *
 * 如有需要，可以覆盖许多"默认"的VisionPortal和AprilTag配置参数。
 * 这些默认参数在下面的代码中以注释形式显示。
 *
 * 使用Android Studio复制此类，并将其粘贴到团队的代码文件夹中，并重命名。
 * 删除或注释掉@Disabled行，将此OpMode添加到驱动站OpMode列表中。
 */
@TeleOp(name = "Concept: AprilTag", group = "Concept")
//@Disabled
public class ConceptAprilTag extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true表示使用 webcam，false表示使用手机相机

    //用于存储AprilTag处理器实例的变量。
    private AprilTagProcessor aprilTag;

    //用于存储视觉门户实例的变量。
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        initAprilTag();

        // 等待驱动站的开始按钮被触碰。
        telemetry.addData("驱动站预览开关", "3个点，相机流");
        telemetry.addData(">", "触碰START开始OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryAprilTag();

                // 将遥测数据推送到驱动站。
                telemetry.update();

                //节省CPU资源；需要时可以恢复流传输。
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // 共享CPU资源。
                sleep(20);
            }
        }

        // 当不再需要相机时，节省更多CPU资源。
        visionPortal.close();

    }   //结束runOpMode()方法

    // 初始化AprilTag处理器。
    private void initAprilTag() {

        // 创建AprilTag处理器。
        aprilTag = new AprilTagProcessor.Builder()

                // 以下默认设置可以取消注释并根据需要进行编辑。
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == 相机校准 ==
                // 如果不手动指定校准参数，SDK将尝试
                // 为您的相机加载预定义的校准。
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... 这些参数分别是fx、fy、cx、cy。

                .build();

        // 调整图像抽取率以权衡检测范围和检测速率。
        // 例如：使用Logitech C920 webcam的一些典型检测数据
        // 抽取率 = 1 .. 以10帧/秒的速度从10英尺外检测2英寸的标签
        // 抽取率 = 2 .. 以22帧/秒的速度从6英尺外检测2英寸的标签
        // 抽取率 = 3 .. 以30帧/秒的速度从4英尺外检测2英寸的标签（默认）
        // 抽取率 = 3 .. 以30帧/秒的速度从10英尺外检测5英寸的标签（默认）
        // 注意：可以在比赛过程中动态更改抽取率以适应情况。
        //aprilTag.setDecimation(3);

        // 使用构建器创建视觉门户。
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // 设置相机（webcam vs. 内置RC手机相机）。
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // 选择相机分辨率。并非所有相机都支持所有分辨率。
        //builder.setCameraResolution(new Size(640, 480));

        // 启用RC预览（LiveView）。设置为"false"可省略相机监控。
        //builder.enableLiveView(true);

        // 设置流格式；MJPEG比默认的YUY2使用更少的带宽。
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // 选择当没有处理器启用时LiveView是否停止。
        // 如果设置为"true"，当没有处理器启用时，监控器显示纯橙色屏幕。
        // 如果设置为"false"，监控器显示没有注释的相机视图。
        builder.setAutoStopLiveView(true);

        // 设置并启用处理器。
        builder.addProcessor(aprilTag);

        // 使用上述设置构建视觉门户。
        visionPortal = builder.build();

        // 可以随时禁用或重新启用aprilTag处理器。
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // 结束initAprilTag()方法

    /// 添加关于AprilTag检测的遥测信息。
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // 遍历检测列表并为每个检测显示信息。
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f （英寸）", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f （度）", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f （英寸，度，度）", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) 未知", detection.id));
                telemetry.addLine(String.format("中心 %6.0f %6.0f   （像素）", detection.center.x, detection.center.y));
            }
        }   // 结束for()循环

        // 向遥测添加"关键"信息
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");//俯仰角、横滚角和偏航角（XYZ 旋转）
        telemetry.addLine("RBE = Range, Bearing & Elevation");//射程、方位角和仰角

    }   // 结束telemetryAprilTag()方法

}   // end class
