package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;

public class OdometryPosition {
    private final DcMotor OY;
    private final DcMotor OX;

    private final double YMOD = 1; //set these to the scaling
    private final double XMOD = 1; //set these to the scaling

    //output values
    public static double X;
    public static double Y;
    public static double W;

    //Odo Offset Values (inches)
    double[] OYoffset = {3,1.5};
    double[] OXoffset = {6.5,0};

    //probably useful junk (i doubt even think god knows what these are used for)
    private double CX;
    private double CY;
    private double PX;
    private double PY;
    private double DX;
    private double DY;
    private int exittimer = 0;
    public boolean exit;
    public double D;

    public OdometryPosition (DcMotor OdoY, DcMotor OdoX, double SY, double SX, double SW) {
        OY = OdoY;
        OX = OdoX;
        Y = SY;//bau
        X = SX;
        W = SW;
    }

    public void OdoCalc () {
        CY = OY.getCurrentPosition(); //i'm not typing in getCurrentPosition() like seven times
        CX = OX.getCurrentPosition(); //wasting precious mem storage just because my fingers are lazy breh

        DY = (CY-PY)/YMOD; //forward odo change
        DX = (CX-PX)/XMOD; //strafe odo change

        //timer for no change in forward
        if (Math.abs(DY) < 0.05) {
            exittimer++;
        } else {
            exittimer = 0;
        }

        PY = CY; //X position
        PX = PY; //Y position

        //yeah math! (this junk looks like magical nonsense now i legit forgot how ts works)
        double FD = Math.hypot(Math.abs(DY), Math.abs(DX));
        double FA = Math.atan2(Math.hypot(OYoffset[0],OYoffset[1])*DY,Math.hypot(OXoffset[0],OXoffset[1])*DX);
        W += -Math.toDegrees(FA);

        //outputs
        W = (W+180)%360 - 180;
        Y += FD*Math.sin(FA);
        X += FD*Math.cos(FA);
        D = FD;

        //no change in forward exit output
        if (exittimer > 500) {
            exit = true;
        } else {
            exit = false;
        }
    }


}
