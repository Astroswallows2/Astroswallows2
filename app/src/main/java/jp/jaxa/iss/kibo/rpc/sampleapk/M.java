package jp.jaxa.iss.kibo.rpc.sampleapk;

import gov.nasa.arc.astrobee.types.Quaternion;

public class M {
    public static boolean isZero(double d){
        double min = 0.00001;
        if(-d > min && d < min){
            return true;
        }else{
            return false;
        }
    }

    //rotate q2 with q1
    public static Quaternion mul(Quaternion q1,Quaternion q2){
        float qx,qy,qz,qw;
        qx=q1.getX()*q2.getW() + q1.getW()*q2.getX() + q1.getY()*q2.getZ() - q1.getZ()*q2.getY();
        qy=q1.getY()*q2.getW() + q1.getW()*q2.getY() + q1.getZ()*q2.getX() - q1.getX()*q2.getZ();
        qz=q1.getZ()*q2.getW() + q1.getW()*q2.getZ() + q1.getX()*q2.getY() - q1.getY()*q2.getX();
        qw=q1.getW()*q2.getW() - q1.getX()*q2.getX() - q1.getY()*q2.getY() - q1.getZ()*q2.getZ();
        return new Quaternion(qx,qy,qz,qw);
    }

    //rotation q from q2 to q1
    public static Quaternion diff(Quaternion q1,Quaternion q2){
        float qx,qy,qz,qw;
        qx=q1.getX()*q2.getW() - q1.getW()*q2.getX() - q1.getY()*q2.getZ() + q1.getZ()*q2.getY();
        qy=q1.getY()*q2.getW() - q1.getW()*q2.getY() - q1.getZ()*q2.getX() + q1.getX()*q2.getZ();
        qz=q1.getZ()*q2.getW() - q1.getW()*q2.getZ() - q1.getX()*q2.getY() + q1.getY()*q2.getX();
        qw=q1.getW()*q2.getW() + q1.getX()*q2.getX() + q1.getY()*q2.getY() + q1.getZ()*q2.getZ();
        return new Quaternion(qx,qy,qz,qw);
    }

    public static Quaternion rev(Quaternion q){
        float qx,qy,qz,qw;
        qx = -q.getX();
        qy = -q.getY();
        qz = -q.getZ();
        qw = q.getW();
        return new Quaternion(qx,qy,qz,qw);
    }

    public static Quaternion vec2quat(double vec[]){
        float qx,qy,qz,qw;
        qx = (float)vec[0];
        qy = (float)vec[1];
        qz = (float)vec[2];
        qw = 0.0f;
        return new Quaternion(qx,qy,qz,qw);
    }

    public static double[] quat2vec(Quaternion q){
        double vec[] = new double[3];
        vec[0] = (double)q.getX();
        vec[1] = (double)q.getY();
        vec[2] = (double)q.getZ();
        return vec;
    }

    //rotate a vector by q.
    public static double[] rotateVec(double vec[],Quaternion q){
        Quaternion q_v,q_rev,q1,q2;
        q_v = vec2quat(vec);
        q_rev = rev(q);
        q1 = mul(q_v,q_rev);
        q2 = mul(q,q1);
        double vec2[] = quat2vec(q2);
        return vec2;
    }

    public static double[] addVec(double vec1[],double vec2[]){
        double vec3[] = new double[3];
        vec3[0] = vec1[0]+vec2[0];
        vec3[1] = vec1[1]+vec2[1];
        vec3[2] = vec1[2]+vec2[2];
        return vec3;
    }

    public static double[] diffVec(double vec1[],double vec2[]){
        double vec3[] = new double[3];
        vec3[0] = vec1[0]-vec2[0];
        vec3[1] = vec1[1]-vec2[1];
        vec3[2] = vec1[2]-vec2[2];
        return vec3;
    }

    //Calculate inner product.
    public static double innerProd(double vec1[],double vec2[]){
        double vec3 = vec1[0]*vec2[0] + vec1[1]*vec2[1] + vec1[2]*vec2[2];
        return vec3;
    }

    //Calculate cross product.
    public static double[] crossProd(double vec1[],double vec2[]){
        double vec3[] = new double[3];
        vec3[0] = vec1[1]*vec2[2] - vec1[2]*vec2[1];
        vec3[1] = vec1[2]*vec2[0] - vec1[0]*vec2[2];
        vec3[2] = vec1[0]*vec2[1] - vec1[1]*vec2[0];
        return vec3;
    }

    //Calculate norm of a vector.
    public static double normVec(double vec[]){
        double norm = Math.sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
        return norm;
    }

    public static double[] normalize(double vec[]){
        double norm = normVec(vec);
        if(isZero(norm)){
            double[] v_normal = {0,0,0};
            return v_normal;
        }else{
            double[] v_normal = scalDiv(vec,norm);
            return v_normal;
        }
    }


    //Multiply a vector by a scalar.
    public static double[] scalMul(double vec[],double k){
        double vec3[] = new double[3];
        vec3[0] = vec[0]*k;
        vec3[1] = vec[1]*k;
        vec3[2] = vec[2]*k;
        return vec3;
    }

    //Divide a vector by a scalar.
    public static double[] scalDiv(double vec[],double k){
        double vec3[] = new double[3];
        vec3[0] = vec[0]/k;
        vec3[1] = vec[1]/k;
        vec3[2] = vec[2]/k;
        return vec3;
    }

    //Calculate a rotation q from vec1 to vec2
    public static Quaternion diffv2v(double vec1[],double vec2[]){
        double v12[] = M.crossProd(vec1,vec2);

        double e[] = M.scalDiv(v12, M.normVec(v12));
        double f = Math.acos((M.innerProd(vec1,vec2))/(M.normVec(vec1)* M.normVec(vec2)));

        android.util.Log.e("AstroSwallows", "[mathCalculation] diffv2v.e="+java.util.Arrays.toString(e));
        String f_val = String.valueOf(f);
        android.util.Log.e("AstroSwallows", "[mathCalculation] diffv2v.f="+f_val);

        double q_v[] = M.scalMul(e,Math.sin(f/2));
        return  new Quaternion((float)q_v[0],(float)q_v[1],(float)q_v[2],(float)Math.cos(f/2));

    }

    //Calculate a rotation q from vec1 to vec2
    public static Quaternion diffv2v_2(double vec1[],double vec2[]){
        vec1 = normalize(vec1);
        vec2 = normalize(vec2);
        double q[] = new double[4];
        double epsilon = 0.0002;

        double c[] = M.crossProd(vec2,vec1);
        double d = normVec(c);
        c = normalize(c);
        double ip = M.innerProd(vec2,vec1);


        if(d<epsilon || 1.0 < ip){
            if(ip < (epsilon-1.0)){
                double vec1_1[] = {-vec1[1],vec1[2],vec1[0]};
                c = normalize(M.crossProd(vec2,vec1));
                q[0] = c[0];
                q[1] = c[1];
                q[2] = c[2];
                q[3] = 0.0;
            }else{
                q[0] = 0.0;
                q[1] = 0.0;
                q[2] = 0.0;
                q[3] = 1.0;
            }
        }else{
            double e[] = scalMul(c,Math.sqrt(0.5*(1-ip)));
            q[0] = e[0];
            q[1] = e[1];
            q[2] = e[2];
            q[3] = Math.sqrt(0.5*(1+ip));
        }

        return  new Quaternion((float)q[0],(float)q[1],(float)q[2],(float)q[3]);

    }


}