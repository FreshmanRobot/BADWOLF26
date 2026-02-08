package org.firstinspires.ftc.teamcode.subsystems;

public class MecanumCalc {

    //pid (P,I,D,ITotal,DPrev)    (what happened to f?)
    private static double[] YPower = {1,0,0,0,0};
    private static double[] XPower = {1,0,0,0,0};
    private static double[] WPower = {1,0,0,0,0};

    //send out (oooh)
    public static double bLPower;
    public static double bRPower;
    public static double fLPower;
    public static double fRPower;
    public static int status;
    public static double SX = 20;
    public static double SY = 122;
    public static double SW = -135;

    //list of points (bruh wha    you forgot to put the points)
    private static double[][] points = {};
    private void GenPoints (String side) {
        if (side == "RED") {
            SX = 144-SX;
            SW = 45;
        }

        shootPos(side);
        pointCreator(44,84,0,0,side);
        pointCreator(10,84,0,1,side);
        pointCreator(14,72,90,0,side);
        shootPos(side);
        pointCreator(44,57,0,0,side);
        pointCreator(10,57,0,1,side);
        shootPos(side);
        pointCreator(44,36,0,0,side);
        pointCreator(10,36,0,1,side);
        shootPos(side);
    }
    private void GenPointsS () {
        SX = 0;
        SY = 0;
        SW = 0;
        for (int a = 0; a < 5; a++) {
            pointCreator(0, 48, 0, 0, "");
            pointCreator(0, 0, 0, 0, "");
        }
    }

    private void GenPointsT () {
        SX = 0;
        SY = 0;
        SW = 0;
        for (int a = 0; a < 5; a++) {
            pointCreator(36, 48, -60, 0, "");
            pointCreator(-36, 48, -180, 0, "");
            pointCreator(0, 0, 30, 0, "");
        }
    }


    //random junk i think
    private static int iter = 0;
    //forgot what these did
    private static final double transError = 1;
    private static final double rotError = 1;
    private static int time = 0;
    public static boolean ledon = false;

    public MecanumCalc (String side) {
        switch (side) {
            case "BLUE":
                GenPoints(side);
                break;
            case "RED":
                GenPoints(side);
                break;
            case "STRAIGHT":
                GenPointsS();
                break;
            case "TRIANGLE":
                GenPointsT();
                break;
            default:

        }
    }

    public static void calculate(double X, double Y, double W, boolean finished, boolean exit) {
        //set up given values
        double TX = points[iter][0];
        double TY = points[iter][1];
        double TW = points[iter][2];
        double IN = points[iter][3];

        //set up derived values
        double DX = TX-X;
        double DY = TY-Y;
        double DW = TW-W;

        //robot is close enough to target or hasn't moved forward in a while unless it is about to shoot
        if ((Math.abs(DX) < transError && Math.abs(DY) < transError && Math.abs(DW) < rotError) || (exit && IN!=2 && points[iter+1][3]!=2)) {
            //arrived at destination
            if (IN != 2 || finished) {
                iter++;
            }
            if (iter >= points.length) {
                //finished
                bLPower = 0;
                bRPower = 0;
                fLPower = 0;
                fRPower = 0;
                status = 0;
            }
        } else {
            //has not arrived at destination

            //full distance
            double FD = Math.hypot(Math.abs(DY), Math.abs(DX));
            //translational error
            //strafe
            double EX = FD*Math.asin(Math.toRadians(DW));
            //forward
            double EY = FD*Math.acos(Math.toRadians(DW));

            //set to minimum rotation
            double UA;
            double BA;
            double MA;
            int dirA;
            if (TW * W < 0) {
                //if flips across horizontal axis
                if (TW > 0) {
                    //target is in upper portion
                    UA = 180-TW;
                    BA = 180+W;
                    dirA = -1;
                } else {
                    //target is in lower portion
                    UA = 180-W;
                    BA = 180+TW;
                    dirA = 1;
                }
                MA = UA+BA;
                if (MA < Math.abs(DW)) {
                    //set new minimum
                    DW = dirA*MA;
                }
            }

            //pid (the f can go f itself)
            WPower[3] += DW;
            YPower[3] += EY;
            XPower[3] += EX;
            DW = WPower[0]*DW + WPower[1]*WPower[3] + WPower[2]*(DW-WPower[4]);
            EY = YPower[0]*EY + YPower[1]*YPower[3] + YPower[2]*(EY-YPower[4]);
            EX = XPower[0]*EX + XPower[1]*XPower[3] + XPower[2]*(EX-XPower[4]);
            WPower[4] = DW;
            YPower[4] = EY;
            XPower[4] = EX;

            //*wide load reversing beep beep beeep*
            time++;
            if (EY < 0) {
                if (time %100 == 0) {
                    ledon = !ledon;
                }
            } else {
                ledon = false;
            }

            //rotational wheel calc
            bLPower = DW;
            bRPower = -DW;
            fLPower = DW;
            fRPower = -DW;

            //strafe wheel calc
            bLPower += -EX;
            bRPower += EX;
            fLPower += EX;
            fRPower += -EX;

            //forward wheel calc
            bLPower += EY;
            bRPower += EY;
            fLPower += EY;
            fRPower += EY;

            //normalize to -1 to 1 (yap yap yap)
            double ratio;
            if (Math.abs(bLPower)*3 > Math.abs(bRPower)+Math.abs(fLPower)+Math.abs(fRPower)) {
                ratio = 1/Math.abs(bLPower);
            } else if (Math.abs(bRPower)*3 > Math.abs(bLPower)+Math.abs(fLPower)+Math.abs(fRPower)) {
                ratio = 1/Math.abs(bRPower);
                //no ones gonna see this comment lol   hehehe
            } else if (Math.abs(fLPower)*3 > Math.abs(bRPower)+Math.abs(bLPower)+Math.abs(fRPower)) {
                ratio = 1/Math.abs(fLPower);
            } else {
                ratio = 1/Math.abs(fRPower);
            }
            //bruh
            if (ratio > 1) {
                ratio = 1;
            }

            //finalized results
            bLPower *= ratio;
            bRPower *= ratio;
            fLPower *= ratio;
            fRPower *= ratio;
            status = (int) IN;
            //yippee
        }
    }

    //just so i can make adding points easier
    private double[][] arrayMod (double[][] in, double[] alter) {
        //set storage array
        double[][] out = new double[in.length+1][alter.length];
        //populate array
        for(int a = 0; a < in.length; a++) {
            System.arraycopy(in[a], 0, out[a], 0, alter.length);
        }
        //append alter
        System.arraycopy(alter, 0, out[in.length], 0, alter.length);
        //exit
        return(out);
    }
    //bluduzz i probably did *not* need two freaking functions for ts
    private void pointCreator (double x, double y, double w, double mPower, String side) {
        //make array of features
        double[] pointGroup = {x,y,w,mPower};
        if (side == "RED") {
            //x pos mirrored
            pointGroup[0] = 144-x;
            //angle needs to be mirrored (how to do?)
            pointGroup[2] = w;
        }
        //call function
        points = arrayMod (points, pointGroup);
    }
    //and i definitely didnt need three functions
    private void shootPos (String side) {
        double shootX = 48;
        double shootY = 96;
        double shootW = 135;

        pointCreator(shootX,shootY,shootW,0, side);
        pointCreator(shootX,shootY,shootW,2, side);
    }
}
