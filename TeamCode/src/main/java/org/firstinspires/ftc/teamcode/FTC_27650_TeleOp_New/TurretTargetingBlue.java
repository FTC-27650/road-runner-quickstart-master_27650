package org.firstinspires.ftc.teamcode.FTC_27650_TeleOp_New;

/**
 * FTC炮台朝向目标点控制类（适配新角度定义：x右0°，y正90°，x左±180°）
 * 适配麦克纳轮底盘，车辆朝向角度由场地实际坐标计算得出
 */
public class TurretTargetingBlue {
    // ************************* 常量定义 *************************
    // 目标点A的坐标 (x: -150cm, y: -141cm)，单位与车辆坐标一致
    private static final double TARGET_X = -150.0;
    private static final double TARGET_Y = -141.0;
    // 炮台旋转限位：左-60°，右60°（机械最大角度，相对于车体）
    private static final double TURRET_MAX_LEFT = -55.0;
    private static final double TURRET_MAX_RIGHT = 55.0;

    // ************************* 实时数据（从场地定位获取） *************************
    private double carX = 0.0; // 车辆当前x坐标（cm）
    private double carY = 0.0; // 车辆当前y坐标（cm）
    private double carHeading = 0.0; // 车辆朝向角度（新规则：x右0°，y正90°，范围[-180, 180]）

    // ************************* 核心计算方法：获取炮台需旋转的角度 *************************

    /**
     * 计算炮台相对于车体的旋转角度（已做归一化和限位处理）
     *
     * @return 炮台目标角度（度）：负=左转，正=右转
     */
    public double getTurretTargetAngle() {
        // 步骤1：计算车辆到目标点的向量差（Δx=目标x-车辆x，Δy=目标y-车辆y）
        double deltaX = TARGET_X - carX;
        double deltaY = TARGET_Y - carY;

        // 步骤2：计算目标点的绝对方位角（新规则：x轴正方向为0°，弧度转角度）
        // Math.atan2(dy, dx)：返回向量(dx, dy)相对于x轴正方向的弧度角，范围[-π, π]
        double thetaRadian = Math.atan2(deltaY, deltaX);
        double thetaDegree = Math.toDegrees(thetaRadian); // 转换为角度，范围[-180, 180]

        // 步骤3：计算炮台原始角度（目标方位角 - 车辆朝向角度）
        double rawAngle = thetaDegree - carHeading;

        // 步骤4：角度归一化（限制在[-180, 180]度，处理360°周期）
        double normalizedAngle = normalizeAngle(rawAngle);

        // 步骤5：机械限位处理（限制在-60~60°）
        double limitedAngle = Math.max(TURRET_MAX_LEFT, Math.min(TURRET_MAX_RIGHT, normalizedAngle));

        return limitedAngle;
    }

    // 辅助方法：角度归一化到[-180, 180]度
    private double normalizeAngle(double angle) {
        angle = angle % 360.0; // 先取余360，得到[-360, 360]
        if (angle > 180.0) {
            angle -= 360.0;
        } else if (angle < -180.0) {
            angle += 360.0;
        }
        return angle;
    }

    // ************************* 数据更新方法（需对接实际定位） *************************

    /**
     * 更新车辆实时坐标（从里程计/视觉定位（如AprilTag）获取后调用）
     *
     * @param x 车辆当前x坐标
     * @param y 车辆当前y坐标
     */
    public void updateCarPosition(double x, double y) {
        this.carX = x;
        this.carY = y;
    }

    /**
     * 更新车辆朝向角度（由场地实际坐标计算得出，需符合新角度规则）
     *
     * @param heading 车辆朝向角度（x右0°，y正90°，范围[-180, 180]）
     */
    public void updateCarHeading(double heading) {
        this.carHeading = heading;
    }
}