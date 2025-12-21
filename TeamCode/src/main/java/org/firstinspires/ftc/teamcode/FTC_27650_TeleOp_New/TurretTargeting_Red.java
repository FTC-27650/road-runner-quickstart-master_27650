package org.firstinspires.ftc.teamcode.FTC_27650_TeleOp_New;

/**
 * FTC炮台朝向目标点控制类（简洁版）
 * 适配麦克纳轮底盘，IMU角度定义：朝y正为0，左转-180~0，右转0~180
 */
public class TurretTargeting_Red {
    // ************************* 常量定义 *************************
    // 目标点A的坐标 (x: -150cm, y: -141cm)，单位与车辆坐标一致
    private static final double TARGET_X = -150.0;
    private static final double TARGET_Y = -141.0;
    // 炮台旋转限位：左-60°，右60°（机械最大角度）
    private static final double TURRET_MAX_LEFT = -60.0;
    private static final double TURRET_MAX_RIGHT = 60.0;

    // ************************* 实时数据（示例，需根据硬件替换） *************************
    // 车辆当前坐标（x/y，单位：cm），需从里程计/视觉定位获取
    private double carX = 0.0;
    private double carY = 0.0;
    // IMU测得的车辆朝向角度（单位：度），需从IMU传感器获取
    private double imuAngle = 0.0;

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

        // 步骤2：计算目标点的绝对方位角（相对于世界y轴正方向，弧度转角度）
        // Java的Math.atan2参数是(纵坐标, 横坐标)，对应我们的(Δx, Δy)，因为我们需要的是相对于y轴的角度
        double thetaRadian = Math.atan2(deltaX, deltaY);
        double thetaDegree = Math.toDegrees(thetaRadian); // 转换为角度（范围：-180~180）

        // 步骤3：计算炮台原始角度（目标方位角 - 车辆IMU角度）
        double rawAngle = thetaDegree - imuAngle;

        // 步骤4：角度归一化（限制在-180~180°，处理360°周期）
        double normalizedAngle = normalizeAngle(rawAngle);

        // 步骤5：机械限位处理（限制在-60~60°）
        double limitedAngle = Math.max(TURRET_MAX_LEFT, Math.min(TURRET_MAX_RIGHT, normalizedAngle));

        return limitedAngle;
    }

    // 辅助方法：角度归一化到[-180, 180]度
    private double normalizeAngle(double angle) {
        angle = angle % 360.0; // 先取余360，得到0~360或-360~0
        if (angle > 180.0) {
            angle -= 360.0;
        } else if (angle < -180.0) {
            angle += 360.0;
        }
        return angle;
    }

    // ************************* 数据更新方法（需根据硬件实现） *************************

    /**
     * 更新车辆实时坐标（从里程计/视觉定位获取后调用）
     *
     * @param x 车辆当前x坐标
     * @param y 车辆当前y坐标
     */
    public void updateCarPosition(double x, double y) {
        this.carX = x;
        this.carY = y;
    }

    /**
     * 更新IMU测得的车辆朝向角度
     *
     * @param angle IMU角度（需保证：朝y正为0，左转负，右转正，范围-180~180）
     */
    public void updateImuAngle(double angle) {
        this.imuAngle = angle;
    }
}