package jp.jaxa.iss.kibo.rpc.sampleapk;

        import android.graphics.Bitmap;
        import android.util.Log;
        import com.google.zxing.BinaryBitmap;
        import com.google.zxing.ChecksumException;
        import com.google.zxing.FormatException;
        import com.google.zxing.LuminanceSource;
        import com.google.zxing.NotFoundException;
        import com.google.zxing.RGBLuminanceSource;
        import com.google.zxing.common.HybridBinarizer;
        import com.google.zxing.qrcode.QRCodeReader;

        import org.jetbrains.annotations.Contract;
        import org.opencv.core.Mat;

        import java.util.Arrays;
        import java.util.regex.Matcher;
        import java.util.regex.Pattern;

        import gov.nasa.arc.astrobee.Result;
        import gov.nasa.arc.astrobee.types.Point;
        import gov.nasa.arc.astrobee.types.Quaternion;
        import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1() {
        // astrobee is undocked and the mission starts
        api.startMission();
        Mat cameraMatrix =AR.makeCamMat();
        Mat distortionCoefficients = AR.makeDistCoef();
        Point pointA1 = new Point(10.3,-9.8,4.5);
        Quaternion quatA1 = new Quaternion(0,0,0,1);
        Point pointA2 = new Point(11.35,-10,4.5);
        Quaternion quatA2 = new Quaternion(0,0,-0.707f,0.707f);
        Point pointA3 = new Point(11.35,-10,4.95);
        Quaternion quatA3 = new Quaternion(0,0,-0.707f,0.707f);

        moveToWrapper2(pointA1,quatA1);

        //pointAに二段階で移動
        moveToWrapper2(pointA2,quatA2);
        moveToWrapper2(pointA3,quatA3);

        Log.e("bmp1_1", "Start ZXing QR reading");
        String valueX = readQRcodeWrapper();
        Log.e("bmp1_1", "Finished ZXing QR reading. String valueX is[" + valueX + "]");
        api.sendDiscoveredQR(valueX);
        Log.e("bmp1_1", "QR code valueX has sent.");
        double[] pxyz = qrtopxyz(valueX);
        Log.e("pattern and pointAA", "[pattern, x, y, z]=" + Arrays.toString(pxyz));

        //QRから読み取った文字列から数値のみ抜き出してdoubleに変換
        int p = (int) pxyz[0];
        double aax = pxyz[1];
        double aay = pxyz[2];
        double aaz = pxyz[3];
        Log.e("pattern and pointAA x y z", "pattern:" + p + "[x:" + aax + "y:" + aay + "z:" + aaz + "]");

        //A'への移動経路
        pathplan(p,aax,aay,aaz,pointA3,quatA3);

        //ARからターゲットまでの距離を考えて補正
        double[] stu = adjustment(p, aax, aaz);
        double axa = stu[0];
        double aya = aay - 0.2;
        double aza = stu[1];

        //read ARcode
        Log.e("start read ARcode", "");
        //ターゲットの中心座標を４つのARの位置から求める
        double[] target_center = readARcode();
        Log.e("finished reading ARcode", Arrays.toString(target_center));
        //opencvの座標系からastrobeeの座標系に変換
        target_center = change_origin(target_center);
        Log.e("finished changing the origin", Arrays.toString(target_center));
        double[] navcam = {0.1177, -0.0422, -0.0826};
        double[] laser = {0.1302, 0.0572, -0.1111};
        double[] beam = {1, 0, 0};
        //レーザーをターゲット中心に向けるクォータニオンq_target
        Log.e("calculate q_target", "");
        Quaternion q_target = rotatetotarget(target_center, navcam, laser, beam);

        //1つのARタグからターゲット中心の座標とクォータニオンを求める処理を書く．
        String del = ":";
        Log.e("finish calculating q_target", String.valueOf(q_target.getX()) + del + String.valueOf(q_target.getY()) + del + String.valueOf(q_target.getZ()) + del + String.valueOf(q_target.getW()));
        relativemoveToWrapper(0,0,0, q_target.getX(), q_target.getY(), q_target.getZ(), q_target.getW());
        Log.e("finish rotating", "");
        //Log.e("finish read ARcode", rotationMat.dump());
        //int corners = readARcode2();
        //Log.e("corners", String.valueOf(corners));
        /*
        String[][] EulerAngles0_s = new String[3][3];
        EulerAngles0_s[1][0] = String.valueOf(target_center[1][0]);
        Log.e("rotaionMatrix2", EulerAngles0_s[1][0]);
        //なんかおかしい
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                EulerAngles0_s[i][j] = String.valueOf(target_center[i][j]);
                Log.e("rotaionMatrix2", EulerAngles0_s[i][j]);
                Log.e("i", String.valueOf(i));
                Log.e("j", String.valueOf(j));
            }
        }


        String[] EulerAngles0_s2 = new String[EulerAngles0_s.length];
        for (int i = 0; i < EulerAngles0_s.length; i++) {
            EulerAngles0_s2[i] = TextUtils.join(",", EulerAngles0_s[i]);
            Log.e("rotaionMatrix2", EulerAngles0_s2[i]);
        }
        String EulerAngles0_s3 = TextUtils.join(",", EulerAngles0_s2);
        //moveToWrapper(aax, aay, aaz,0, 0, -0.7071068,0.7071068);
        //moveToA''
        Log.e("rotationMatrix_all", EulerAngles0_s3);
        */
        /*
        moveToWrapper(axa, aya, aza, 0, 0, -0.7071068, 0.7071068);
        Log.e("move to AA", "bee finished moving to pointAA.");
        */
        // irradiate the laser


/*
        Mat ARcode = readARcode2();
        String zero = ARcode.toString();
        Log.e("ARcode Image", zero);
        */
        Log.e("Laser", "Bee start irradiating the laser.");
        api.laserControl(true);

        // Take a snapshot
        Log.e("Snapshot", "Bee start taking a snapshot .");
        api.takeSnapshot();
        Log.e("Snapshot", "Bee finished taking a snapshot .");

        // turn off the laser
        api.laserControl(false);
        Log.e("Laser", "Bee quit irradiating the laser.");

        //Bへの移動
        pathplan2(p,aax,aay,aaz,pointA3,quatA3);

        // Send mission completion
        api.reportMissionCompletion();
        Log.e("MissionCompletion", "Mission completed.");
    }

    @Override
    protected void runPlan2() {
        // write here your plan 2
    }

    @Override
    protected void runPlan3() {
        // write here your plan 3
    }

    // You can add your method
    private void moveToWrapper(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w) {
        final int LOOP_MAX = 3;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);
        Log.e("Moveto","Bee start moving to x:"+pos_x+" y:"+pos_y+" z:"+pos_z);
        Result result = api.moveTo(point, quaternion, true);

        int loopCounter = 0;

        //条件文&&に変更
        while (!result.hasSucceeded() && loopCounter < LOOP_MAX)
        {
            Log.e("Moveto","Bee start moving to x:"+pos_x+" y:"+pos_y+" z:"+pos_z);
            Log.e("Moveto","loopCounter = "+loopCounter);
            result = api.moveTo(point, quaternion, true);
            ++loopCounter;
        }
    }

    private void moveToWrapper2(Point point,Quaternion quat) {
        final int LOOP_MAX = 3;
        Result result = api.moveTo(point, quat, true);
        int loopCounter = 0;
        while (!result.hasSucceeded() || loopCounter < LOOP_MAX) {
            Log.e("Moveto", "Bee start moving to x:" + point.getX()
                    + " y:" + point.getY() + " z:" + point.getZ());
            Log.e("Loop Counter", "loop counter is" + loopCounter);

            result = api.moveTo(point, quat, true);
            ++loopCounter;
        }
    }

    private void relativemoveToWrapper(double pos_x, double pos_y, double pos_z,
                                       double qua_x, double qua_y, double qua_z,
                                       double qua_w) {
        final int LOOP_MAX = 3;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);

        Result result = api.relativeMoveTo(point, quaternion, true);

        int loopCounter = 0;
        while (result.hasSucceeded() && loopCounter < LOOP_MAX) {
            Log.e("Moveto", "Bee start moving to x:" + pos_x + " y:" + pos_y + " z:" + pos_z);
            Log.e("Loop Counter relative", "loop counter is" + loopCounter);

            result = api.relativeMoveTo(point, quaternion, true);
            ++loopCounter;
        }
    }

    public double[] change_origin(double[] point) {
        double[] changed_point = new double[3];
        changed_point[0] = point[2];
        changed_point[1] = point[0];
        changed_point[2] = point[1];
        return changed_point;
    }

/*
    public double[] change_origin(double[] point) {
        double[] changed_point = new double[3];
        changed_point[0] = point[0];
        changed_point[1] = -point[2];
        changed_point[2] = point[1];
        return changed_point;
    }
    */

    public String readQrcode(Bitmap bitmap) {
        //final int navcamWidth = 1280;
        //final int navcamHeight = 960;
        final int trimStartx = 480;
        final int trimStarty = 500;
        final int trimWidth = 320;
        final int trimHeight = 380;

        //トリミング作業
        Bitmap bitmap_trim = Bitmap.createBitmap(bitmap, trimStartx, trimStarty, trimWidth, trimHeight);
        // Bitmap のサイズを取得して、ピクセルデータを取得する
        int width = bitmap_trim.getWidth();
        Log.d("readQR", "bitmap width=" + width);
        int height = bitmap_trim.getHeight();
        Log.d("readQR", "bitmap height=" + height);
        int[] pixels = new int[width * height];
        Log.e("bmp1_1", "getPixels start.");
        bitmap_trim.getPixels(pixels, 0, width, 0, 0, width, height);
        Log.e("bmp1_1", "getPixels finished.");

        try {
            // zxing で扱える BinaryBitmap形式に変換する
            Log.e("bmp1_1", "RGB start.");
            LuminanceSource source = new RGBLuminanceSource(width, height, pixels);
            Log.e("bmp1_1", "RGB finished.");
            //Log.e("bmp1_1","binarybitmap start.");
            BinaryBitmap binaryBitmap = new BinaryBitmap(new HybridBinarizer(source));
            //Log.e("bmp1_1","binarybitmap finished.");

            // zxing で画像データを読み込み解析する
            //Log.e("bmp1_1","QRcodereader start.");
            QRCodeReader reader = new QRCodeReader();
            //Log.e("bmp1_1","QRcodereader finished.");
            Log.e("bmp1_1", "decord start.");
            com.google.zxing.Result decodeResult = reader.decode(binaryBitmap);
            Log.e("bmp1_1", "decord finished.");

            // 解析結果を取得する
            //Log.e("bmp1_1","gettext start.");
            String result = decodeResult.getText();
            Log.d("readQR", result);
            return result;
        }
        //エラーの原因出力
        catch (NotFoundException e) {
            //QRが見つからなかったとき
            Log.e("readQR", "NotFoundException occur : " + e.getMessage());
            return null;
        } catch (FormatException e) {
            Log.e("readQR", "CheckSumException occur : " + e.getMessage());
            return null;
        } catch (ChecksumException e) {
            Log.e("readQR", "FormatException occur : " + e.getMessage());
            return null;
        }
    }

    private String readQRcodeWrapper() {
        final int LOOP_MAX = 3;
        int loopCounter = 0;
        String result = null;
        Bitmap bitmap;

        while (result == null && loopCounter < LOOP_MAX) {
            Log.e("readQRtry", "loopCounter = " + loopCounter);
            bitmap = api.getBitmapNavCam();
            result = readQrcode(bitmap);
            ++loopCounter;
        }
        return result;
    }

    public static double[] qrtopxyz(String qr) {
        String cutstring = "[{]|[}]|[\"p\":]|[\"x\":]|[\"y\":]|[\"z\":]";
        Pattern cutpattern = Pattern.compile(cutstring);
        Matcher m = cutpattern.matcher(qr);
        qr = m.replaceAll("");
        String[] stringpxyz = qr.split(",");
        double[] pxyz = new double[4];
        for (int i = 0; i < 4; i++) {
            pxyz[i] = Double.parseDouble(stringpxyz[i]);
        }
        return pxyz;
    }

    //@org.jetbrains.annotations.Contract(pure = true)
    @Contract("null -> fail; !null -> !null")
    public static double[] adjustment(int num1, double num2, double num4) {
        double[] stu = new double[2];
        if ((num1 == 1) || (num1 == 8)) {
            double num22 = num2 - 0.1125;
            //double aya = num3;
            double num44 = num4 + 0.0415;
            stu[0] = num22;
            stu[1] = num44;
        } else if ((num1 == 2) || (num1 == 3) || (num1 == 4)) {
            double num22 = num2 + 0.1125;
            //double aya = num3;
            double num44 = num4 + 0.0415;
            stu[0] = num22;
            stu[1] = num44;
        } else if ((num1 == 5) || (num1 == 6)) {
            double num22 = num2 + 0.1125;
            //double aya = num3;
            double num44 = num4 - 0.0415;
            stu[0] = num22;
            stu[1] = num44;
        } else {
            double num22 = num2 - 0.1125;
            //double aya = num3;
            double num44 = num4 - 0.0415;
            stu[0] = num22;
            stu[1] = num44;
        }
        return stu;
    }
    
    //A'への経路パターン別
    //adx,ady,adzがA’の座標pxyz[]
    private void pathplan(int p, double adx, double ady, double adz, Point pointA,Quaternion quatA) {
        Point pointAd = new Point(adx,ady,adz);
        if ((p == 1) || (p == 2) ||(p == 8)) {
            moveToWrapper2(pointAd,quatA);
        } else if ((p == 3) || (p == 4)) {
            //2段階移動(1回目：xyのみ移動，2回目：zのみ移動)
            Point pointAA = new Point(adx,ady,pointA.getZ());
            moveToWrapper2(pointAA,quatA);
            moveToWrapper2(pointAd,quatA);
        } else if ((p == 5) || (p == 6)) {
            Point pointAA1 = new Point(10.8,ady,pointA.getZ());
            Point pointAA2 = new Point(10.8,ady,adz);
            moveToWrapper2(pointAA1,quatA);
            moveToWrapper2(pointAA2,quatA);
            moveToWrapper2(pointAd,quatA);
        } else {
            Point pointAA1 = new Point(11.5,ady,pointA.getZ());
            Point pointAA2 = new Point(11.5,ady,adz);
            moveToWrapper2(pointAA1,quatA);
            moveToWrapper2(pointAA2,quatA);
            moveToWrapper2(pointAd,quatA);
        }
    }

    public double[] readARcode() {
        Mat cameraMatrix = AR.makeCamMat();
        Mat distortionCoefficients = AR.makeDistCoef();
        Mat mat3 = api.getMatNavCam();
        double[] target_center = AR.detectTarget(mat3, cameraMatrix, distortionCoefficients);
        return target_center;
    }

    public String readARcode3() {
        Mat cameraMatrix = AR.makeCamMat();
        Mat distortionCoefficients = AR.makeDistCoef();
        Mat mat3 = api.getMatNavCam();
        String target_center = AR.detectTarget2(mat3, cameraMatrix, distortionCoefficients);
        return target_center;
    }

    public void readARcode4() {
        Mat cameraMatrix = AR.makeCamMat();
        Mat distortionCoefficients = AR.makeDistCoef();
        Mat mat3 = api.getMatNavCam();
        AR.detectTarget3(mat3, cameraMatrix, distortionCoefficients);
    }


    public int readARcode2() {
        Mat cameraMatrix = AR.makeCamMat();
        Mat distortionCoefficients = AR.makeDistCoef();
        Mat mat3 = api.getMatNavCam();
        int corners = AR.MarkerImage(mat3, cameraMatrix, distortionCoefficients);
        return corners;
/*
    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        IntentResult result = IntentIntegrator.parseActivityResult(requestCode, resultCode, data);
        if(result != null) {
            Log.d("readQR", result.getContents());
        } else {
            super.onActivityResult(requestCode, resultCode, data);
        }
    }

 */
    }

    public Quaternion rotatetotarget(double[] target_center, double[] navcam, double[] laser, double[] beam) {
        //ビームの長さk
        target_center = M.addVec(target_center, navcam);
        double k =  Math.sqrt(Math.pow(target_center[0],2) + Math.pow(target_center[1],2) + Math.pow(target_center[2],2)
                - Math.pow(laser[1], 2) - Math.pow(laser[2], 2) - laser[0]);
        Log.e("k", String.valueOf(k));
        //回転前のビームの先端．ターゲット中心ベクトルと大きさ同じ
        double[] target_center_beam = M.addVec(laser, M.scalMul(beam, k));
        Log.e("target_center_beam", Arrays.toString(target_center_beam));
        Log.e("target_center", Arrays.toString(target_center));
        Quaternion q_target = M.diffv2v_2(target_center, target_center_beam);
        return q_target;
    }

    //Bへの経路パターン別1,3,4,5,6
    //adx,ady,adzがA’の座標pxyz[]
    private void pathplan2(int p, double adx, double ady, double adz, Point pointA,Quaternion quatA) {
        Point pointAd = new Point(adx,ady,adz);
        Point pointB = new Point(10.6,-8.0,4.5);
        if ((p == 1)||(p == 8)){
            Point pointAA1 = new Point(adx-0.35,-9.0,adz-0.5);
            Point pointAA2 = new Point(pointAA1.getX()-0.35,-9.0,pointAA1.getZ()-0.1);
            moveToWrapper2(pointAA1,quatA);
            moveToWrapper2(pointAA2,quatA);
            moveToWrapper2(pointB,quatA);
        }else if (p == 2){
            Point pointAA1 = new Point(adx-0.5,ady+0.8,adz);
            moveToWrapper2(pointAA1,quatA);
            moveToWrapper2(pointB,quatA);
        }else if (p == 3){
            Point pointAA1 = new Point(adx-0.5,ady+0.8,adz-0.08);
            moveToWrapper2(pointAA1,quatA);
            moveToWrapper2(pointB,quatA);
        }else if (p == 4){
            Point pointAA1 = new Point(adx-0.5,ady+0.8,adz-0.7);
            moveToWrapper2(pointAA1,quatA);
            moveToWrapper2(pointB,quatA);
        }else if (p == 5) {
            Point pointAA1 = new Point(10.6,-8.8,5.1);
            moveToWrapper2(pointAA1,quatA);
            moveToWrapper2(pointB,quatA);
        }else if (p == 6) {
            Point pointAA1 = new Point(10.6,-8.8,5.25);
            moveToWrapper2(pointAA1,quatA);
            moveToWrapper2(pointB,quatA);
        }else{
            Point pointAA1 = new Point(11.5,ady,adz);
            Point pointAA2 = new Point(pointAA1.getX(),ady,pointA.getZ());
            //Point pointAA3 = new Point(pointAA2.getX()-0.9,pointAA2.getY()+0.8,4.73);
            Point pointAA3 = new Point(10.6,-8.8,4.73);
            moveToWrapper2(pointAA1,quatA);
            moveToWrapper2(pointAA2,quatA);
            moveToWrapper2(pointAA3,quatA);
            moveToWrapper2(pointB,quatA);
        }
    }
}
