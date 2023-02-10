using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using MathNet.Numerics.LinearAlgebra.Single;

namespace InverseKinematics
{
    public class JointController : MonoBehaviour
    {
        private static int n = 6;
        private GameObject[] joint = new GameObject[n];
        private float[] angle = new float[n];
        private Vector3[] dim = new Vector3[n];
        private Vector3[] point = new Vector3[n+1];
        private Vector3[] axis = new Vector3[n];
        private Quaternion[] rotation = new Quaternion[n];
        private Quaternion[] wRotation = new Quaternion[n];

        private Vector3 pos;
        private Vector3 rot;

        private float lambda = 0.1f;

        private GameObject[] slider = new GameObject[n];
        private float[] sliderVal = new float[n];
        private float[] prevSliderVal = new float[n];
        private GameObject[] angText = new GameObject[n];
        private GameObject[] posText = new GameObject[n];

        // private float[] prevAngle = new float[6];
        // private float[] minAngle = new float[6];
        // private float[] maxAngle = new float[6];


        // Start is called before the first frame update
        void Start()
        {
            for(int i=0;i<joint.Length; i++){
                joint[i] = GameObject.Find("Joint_"+i.ToString());
            }

            for(int i=0;i<joint.Length; i++){
                slider[i] = GameObject.Find("Slider_"+i.ToString());
                sliderVal[i] = slider[i].GetComponent<Slider>().value;
                posText[i] = GameObject.Find("Ref_"+i.ToString());
                angText[i] = GameObject.Find("Ang_"+i.ToString());
            }

            dim[0] = new Vector3(0f,2f,0f);
            dim[1] = new Vector3(4f,0f,0f);
            dim[2] = new Vector3(1f,0f,0f);
            dim[3] = new Vector3(2f,0f,0f);
            dim[4] = new Vector3(1f,0f,0f);
            dim[5] = new Vector3(2f,0f,0f);

            axis[0] = new Vector3(0f,1f,0f);
            axis[1] = new Vector3(0f,0f,1f);
            axis[2] = new Vector3(0f,0f,1f);
            axis[3] = new Vector3(1f,0f,0f);
            axis[4] = new Vector3(0f,0f,1f);
            axis[5] = new Vector3(1f,0f,0f);
        
            angle[0] = 0f;
            angle[1] = 90f;
            angle[2] = 0f;
            angle[3] = 0f;
            angle[4] = 0f;
            angle[5] = 0f;
        }

        // Update is called once per frame
        void Update()
        {
            for(int i=0;i<joint.Length;i++) {
                sliderVal[i] = slider[i].GetComponent<Slider>().value;
            }
            pos.x = sliderVal[0];
            pos.y = sliderVal[1];
            pos.z = sliderVal[2];
            rot.x = sliderVal[3];
            rot.y = sliderVal[4];
            rot.z = sliderVal[5];

            CalcIK();
        }

        void CalcIK()
        {
            for(int i=0;i<100;i++)
            {
                ForwardKinematics();
                var err = CalcErr();
                float err_norm = (float)err.L2Norm();
                if(err_norm<1E-3)
                {
                    break;
                }
                var J = CalcJacobian();
                var dAngle  = lambda * J.PseudoInverse() * err;
                for (int ii=0;ii<joint.Length;ii++)
                {
                    angle[ii] += dAngle[ii,0]*Mathf.Rad2Deg;
                }
            }

            for(int i=0;i<joint.Length;i++)
            {
                rotation[i] = Quaternion.AngleAxis(angle[i],axis[i]);
                joint[i].transform.localRotation = rotation[i];
                prevSliderVal[i] = sliderVal[i];
                posText[i].GetComponent<TMPro.TextMeshProUGUI>().text = sliderVal[i].ToString("f2");
                angText[i].GetComponent<TMPro.TextMeshProUGUI>().text = angle[i].ToString("f2");
            }
        }

        void ForwardKinematics()
        {
            point[0] = new Vector3(0f,0f,0f);
            wRotation[0] = Quaternion.AngleAxis(angle[0], axis[0]);
            for(int i=1;i<joint.Length;i++)
            {
                point[i] = wRotation[i-1]*dim[i-1]+point[i-1];
                rotation[i] = Quaternion.AngleAxis(angle[i], axis[i]);
                wRotation[i] = wRotation[i-1]*rotation[i];
            }
            point[joint.Length] = wRotation[joint.Length-1]*dim[joint.Length-1] + point[joint.Length - 1];
        }

        DenseMatrix CalcErr()
        {
            Vector3 perr = pos - point[6];
            Quaternion rerr = Quaternion.Euler(rot)*Quaternion.Inverse(wRotation[5]);

            Vector3 rerrVal = new Vector3(rerr.eulerAngles.x,
                                          rerr.eulerAngles.y,
                                          rerr.eulerAngles.z);
            if(rerrVal.x > 180f) rerrVal.x -= 360f;
            if(rerrVal.y > 180f) rerrVal.x -= 360f;
            if(rerrVal.z > 180f) rerrVal.x -= 360f;
            var err = DenseMatrix.OfArray(new float[,]
            {
                {perr.x},
                {perr.y},
                {perr.z},
                {rerrVal.x * Mathf.Deg2Rad},
                {rerrVal.y * Mathf.Deg2Rad},
                {rerrVal.z * Mathf.Deg2Rad}
            });

            return err;
        }

        DenseMatrix CalcJacobian()
        {
            Vector3 w0 = wRotation[0] * axis[0];
            Vector3 w1 = wRotation[1] * axis[1];
            Vector3 w2 = wRotation[2] * axis[2];
            Vector3 w3 = wRotation[3] * axis[3];
            Vector3 w4 = wRotation[4] * axis[4];
            Vector3 w5 = wRotation[5] * axis[5];

            Vector3 p0 = Vector3.Cross(w0, point[6]-point[0]);
            Vector3 p1 = Vector3.Cross(w1, point[6]-point[1]);
            Vector3 p2 = Vector3.Cross(w2, point[6]-point[2]);
            Vector3 p3 = Vector3.Cross(w3, point[6]-point[3]);
            Vector3 p4 = Vector3.Cross(w4, point[6]-point[4]);
            Vector3 p5 = Vector3.Cross(w5, point[6]-point[5]);

            var J = DenseMatrix.OfArray(new float[,]
            {
                {p0.x, p1.x, p2.x, p3.x, p4.x, p5.x},
                {p0.y, p1.y, p2.y, p3.y, p4.y, p5.y},
                {p0.z, p1.z, p2.z, p3.z, p4.z, p5.z},
                {w0.x, w1.x, w2.x, w3.x, w4.x, w5.x},
                {w0.y, w1.y, w2.y, w3.y, w4.y, w5.y},
                {w0.z, w1.z, w2.z, w3.z, w4.z, w5.z}
            });

            return J;
        }

    }
}