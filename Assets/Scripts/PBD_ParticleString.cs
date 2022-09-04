
using UdonSharp;
using UnityEngine;
using VRC.SDKBase;
using VRC.Udon;

//ぐぬぬ
// public class Constraint {
//     public int i; //質点その1
//     public int j; //質点その2
//     public float d; //定常状態の伸び

//     public Constraint(int i, int j, float d) {
//         this.i = i;
//         this.j = j;
//         this.d = d;
//     }
// }

public class PBD_ParticleString : UdonSharpBehaviour {

    public int n = 24; //質点の個数
    [SerializeField, Range(0, 1)] float k = 0.5f; //バネの硬さ
    public float dt = 0.01f; //Delta Time
    public Vector3 gravity = Vector3.zero; //重力
    public float kDamping = 0.03f; //Velocity Dampingで使用する定数
    public Transform startPoint; //ひもの開始点
    public Transform endPoint; //ひもの終了点
    public ParticleSystem particleSystem;
    ParticleSystem.Particle[] particles;
    Vector3[] x = null; //質点の位置
    Vector3[] v = null; //質点の速度
    Vector3[] p = null; //質点の推定位置
    bool[] isFixed = null;
    float[] m = null; //質量

    //拘束
    int[] i = null; //質点その1
    int[] j = null; //質点その2
    float[] d = null; //定常状態の伸び
    // Constraint[] constraint; //拘束

    void Start() {

        //n個の質点の初期化
        p = new Vector3[n]; //推定位置
        x = new Vector3[n]; //位置
        v = new Vector3[n]; //速度
        m = new float[n]; //質量

        for(int k = 0; k < n; k++) {

            //現在のループ回数を、質点の個数-1で割った数
            float t = (float)(k) / (n-1);

            //位置
            x[k] = Vector3.Lerp(startPoint.position, endPoint.position, t);

            //速度
            v[k] = Vector3.zero;

            //質量
            m[k] = 1f;

        }

        //拘束の初期化
        i = new int[n-1];
        j = new int[n-1];
        d = new float[n-1];
        
        for(int k = 0; k < n - 1; k++) {

            //ベクトルの長さ
            float l = Mathf.Sqrt((x[k].x - x[k+1].x)*(x[k].x - x[k+1].x) + (x[k].y - x[k+1].y)*(x[k].y - x[k+1].y) + (x[k].z - x[k+1].z)*(x[k].z - x[k+1].z));

            i[k] = k;
            j[k] = k+1;
            d[k] = l;
        } 

        //ヒモの両端の質点は固定
        isFixed = new bool[n];
        isFixed[0] = true;
        isFixed[n - 1] = true;

        InitializeParticle();
        
    }

    void Update() {

        //外力による速度変化
        for(int k = 0; k < n; k++) {
            v[k] += gravity *dt;
            if(isFixed[k]) v[k] = Vector3.zero;
        }

        //位置の更新
        for(int k = 0; k < n; k++) {
            p[k] = x[k] + v[k] * dt;
            x[k] = p[k];
        }

        x[0] = startPoint.position;
        x[n-1] = endPoint.position;


        //速度の更新
        for(int ii = 0; ii < i.Length; ii++) {

            var p1 = p[i[ii]];
            var p2 = p[j[ii]];
            float w1 = 1f / m[i[ii]];
            float w2 = 1f / m[j[ii]];
            float diff = Vector3.Magnitude(p1 - p2);
            var dp1 = -k * w1 / (w1+w2) * (diff - d[ii]) * Vector3.Normalize(p1 - p2);
            var dp2 = k * w2 / (w1+w2) * (diff - d[ii]) * Vector3.Normalize(p1 - p2);

            v[i[ii]] += dp1 / dt;
            v[j[ii]] += dp2 / dt;

        }

        SetParticle();

    }

    // //速度のDamping
    // void VelocityDamping(int n, Vector3[] x, Vector3[] v, float[] m, float k) {

    //     var xcm = Vector3.zero;
    //     var vcm = Vector3.zero;
    //     var totalMass = 0f;
    //     for(int i = 0; i < n; i++) {
    //         xcm += x[i];
    //         vcm += v[i];
    //         totalMass += m[i];
    //     }

    //     xcm /= totalMass;
    //     vcm /= totalMass;

    //     var L = Vector3.zero;
    //     var I = new SquareMatrix(3);
    //     var rs = new Vector3[n];
        
    //     for(int i = 0; i < n; i++) {
    //         Vector3 r = x[i] - xcm;
    //         rs[i] = r;

    //         var R = new SquareMatrix(3);
    //         R
    //     }

    // } 

    void SetParticle() {

        particles = new ParticleSystem.Particle[n];
        int num = particleSystem.GetParticles(particles);

        for(int i = 0; i < n; i++) {
            particles[i].position = x[i];
        }

        particleSystem.SetParticles(particles, num);

    }

    private void InitializeParticle() {

        var main = particleSystem.main;
        main.maxParticles = n;

        //自分でインスペクターで入力する
        // var emission = particleSystem.emission;
        // emission.SetBursts(new ParticleSystem.Burst[] { new ParticleSystem.Burst(0f, n) });
        
        particleSystem.Play();

    }
}
