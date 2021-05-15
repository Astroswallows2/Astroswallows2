package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.graphics.Bitmap;
import android.util.Log;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.ChecksumException;
import com.google.zxing.DecodeHintType;
import com.google.zxing.FormatException;
import com.google.zxing.LuminanceSource;
import com.google.zxing.MultiFormatReader;
import com.google.zxing.NotFoundException;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.Reader;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

import org.jetbrains.annotations.Contract;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
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
    protected void runPlan1()
    {
        // astrobee is undocked and the mission starts
        api.startMission();

        // astrobee is undocked and the mission starts
        moveToWrapper(10.3, -9.8, 4.5, 0, 0, 0, 1);
        //api.getTrustedRobotKinematics();

        //pointAに移動
        //moveToWrapper(11.21, -10, 5, 0, 0, -1 / Math.sqrt(2), 1 / Math.sqrt(2));
        moveToWrapper(11.3, -10, 4.5, 0, 0, -1 / Math.sqrt(2), 1 / Math.sqrt(2));
        moveToWrapper(11.3, -10, 5.1, 0, 0, -1 / Math.sqrt(2), 1 / Math.sqrt(2));
        //api.getTrustedRobotKinematics();




        /*追加分*/
        Bitmap bmp1_1 = api.getBitmapNavCam();
        Log.e("bmp1_1","has loaded as BitmapDockCam. Bitmap data bmp1_1 is["+bmp1_1);
        Log.e("bmp1_1","Start ZXing QR reading");
        String valueX = readQrcode(bmp1_1);
        Log.e("bmp1_1","Finished ZXing QR reading. String valueX is["+valueX+"]");
        api.sendDiscoveredQR(valueX);
        Log.e("bmp1_1","QR code valueX has sent.");
        double[] pxyz = qrtopxyz(valueX);
        Log.e("pattern and pointAA","[pattern, x, y, z]="+ Arrays.toString(pxyz));
        int p = (int)pxyz[0];
        double aax = pxyz[1];
        double aay = pxyz[2];
        double aaz = pxyz[3];

        Log.e("pattern and pointAA x y z","pattern:"+String.valueOf(p)+
                "[x:"+String.valueOf(aax)+ "y:"+String.valueOf(aay)+ "z:"+String.valueOf(aaz)+"]");
        //追加はじめ
        double[] stu = adjustment(p,aax,aaz);
        double axa = stu[0];
        //double aya = stu[1];
        double aza = stu[1];
        //追加ここまで

        //moveToWrapper(aax, aay, aaz,0, 0, -0.7071068,0.7071068);
        moveToWrapper(axa, aay, aza,0, 0, -0.7071068,0.7071068);
        Log.e("move to AA","bee finished moving to pointAA.");

        // irradiate the laser]
        Log.e("Laser","Bee start irradiating the laser.");
        api.laserControl(true);
        Log.e("Laser","Bee finished irradiating the laser.");
        // Take a snapshot
        Log.e("Snapshot","Bee start taking a snapshot .");
        api.takeSnapshot();
        Log.e("Snapshot","Bee finished taking a snapshot .");
        // Send mission completion
        api.reportMissionCompletion();
        Log.e("MissionCompletion","Mission completed.");
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
                               double qua_w)
    {
        final int LOOP_MAX = 3;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);

        Result result = api.moveTo(point, quaternion, true);

        int loopCounter = 0;
        while (!result.hasSucceeded() || loopCounter < LOOP_MAX)
        {
            Log.e("Moveto","Bee start moving to x:"+pos_x+" y:"+pos_y+" z:"+pos_z);
            result = api.moveTo(point, quaternion, true);
            ++loopCounter;
        }
    }

    public String readQrcode(Bitmap bitmap)
    {
        String result = null;
        final int navcamWidth = 1280;
        final int navcamHeight = 960;
        final int trimStartx = 512;
        final int trimStarty = 320;
        final int trimWidth = 256;
        final int trimHeight = 380;

        Bitmap bitmap_trim = Bitmap.createBitmap(bitmap, trimStartx,trimStarty,trimWidth,trimHeight);
        // Bitmap のサイズを取得して、ピクセルデータを取得する
        int width = bitmap_trim.getWidth();
        Log.d("readQR", "bitmap width="+width);
        int height = bitmap_trim.getHeight();
        Log.d("readQR", "bitmap height="+height);
        int[] pixels = new int[width * height];
        Log.e("bmp1_1","getPixels start.");
        bitmap_trim.getPixels(pixels, 0, width, 0, 0, width, height);
        Log.e("bmp1_1","getPixels finished.");
        try
        {
            // zxing で扱える BinaryBitmap形式に変換する
            Log.e("bmp1_1","RGB start.");
            LuminanceSource source = new RGBLuminanceSource(width, height, pixels);
            Log.e("bmp1_1","RGB finished.");
            Log.e("bmp1_1","binarybitmap start.");
            BinaryBitmap binaryBitmap = new BinaryBitmap(new HybridBinarizer(source));
            Log.e("bmp1_1","binarybitmap finished.");
            // zxing で画像データを読み込み解析する
            Log.e("bmp1_1","multiformatreader start.");
            QRCodeReader reader = new QRCodeReader();
            Log.e("bmp1_1","multiformatreader finished.");
            Log.e("bmp1_1","decord start.");
            com.google.zxing.Result decodeResult = reader.decode(binaryBitmap);
            Log.e("bmp1_1","decord finished.");
            // 解析結果を取得する
            Log.e("bmp1_1","gettext start.");
            result = decodeResult.getText();
            Log.d("readQR", result);
            return result;
        }
        /*
        catch (Exception e)
        {
            Log.e("readQR", e.getLocalizedMessage());
            return null;
        }
         */
        catch (NotFoundException e)
        {
            Log.e("readQR", "NotFoundException occur : " + e.getMessage());;
            return null;
        } catch (FormatException e) {
            Log.e("readQR", "CheckSumException occur : " + e.getMessage());
            return null;
        } catch (ChecksumException e) {
            Log.e("readQR", "FormatException occur : " + e.getMessage());
            return null;
        }
    }

    public static double[] qrtopxyz(String qr)
    {
        String cutstring = "[{]|[}]|[\"p\":]|[\"x\":]|[\"y\":]|[\"z\":]";
        Pattern cutpattern = Pattern.compile(cutstring);
        Matcher m = cutpattern.matcher(qr);
        qr = m.replaceAll("");
        String[] stringpxyz = qr.split(",");
        double[] pxyz = new double[4];
        for (int i = 0; i < 4; i++)
        {
            pxyz[i] = Double.parseDouble(stringpxyz[i]);
        }
        return pxyz;
    }

    //@org.jetbrains.annotations.Contract(pure = true)
    @Contract("null -> fail; !null -> !null")
    public static double[] adjustment(int num1, double num2, double num4)
    {double[] stu = new double[2];
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
}
