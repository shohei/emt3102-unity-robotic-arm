using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace InverseKinematics
{
    public class JointController : MonoBehaviour
    {
        private static int n = 3;
        private GameObject[] joint = new GameObject[n];
        private GameObject[] arm = new GameObject[n];
        private float[] armL = new float[n];
        private Vector3[] angle = new Vector3[n];

        private GameObject[] slider = new GameObject[n];
        private float[] sliderVal = new float[n];
        private GameObject[] angText = new GameObject[n];
        private GameObject[] posText = new GameObject[n];

        // Start is called before the first frame update
        void Start()
        {
            for(int i=0;i<joint.Length; i++){
                joint[i] = GameObject.Find("Joint_"+i.ToString());
                arm[i] = GameObject.Find("Arm_"+i.ToString());
                if (i==0) armL[i] = arm[i].transform.localScale.y;
                else armL[i] = arm[i].transform.localScale.x;
            }

            for(int i=0;i<joint.Length; i++){
                slider[i] = GameObject.Find("Slider_"+i.ToString());
                sliderVal[i] = slider[i].GetComponent<Slider>().value;
                angText[i] = GameObject.Find("Ref_"+i.ToString());
                posText[i] = GameObject.Find("Ang_"+i.ToString());
            }
        
        }

        // Update is called once per frame
        void Update()
        {
            for(int i=0;i<joint.Length;i++) {
                sliderVal[i] = slider[i].GetComponent<Slider>().value;
            }
            float x = sliderVal[0];
            float y = sliderVal[1];
            float z = sliderVal[2];
            angle[0].y = Mathf.Atan2(z,x);
            float a = x / Mathf.Cos(angle[0].y);
            float b = y - armL[0];
            float alpha = Mathf.Acos((armL[1]*armL[1] + armL[2]*armL[2] - a*a - b*b) / (2f* armL[1] * armL[2]));
            angle[2].z = -Mathf.PI + alpha;
            float beta = Mathf.Acos((armL[1]*armL[1] + a*a + b*b - armL[2]*armL[2]) /  (2f* armL[1] * Mathf.Pow((a*a + b*b), 0.5f)));
            angle[1].z = -Mathf.Atan2(b,a) + beta;
            
            for(int i=0;i<joint.Length; i++){
                joint[i].transform.localEulerAngles = angle[i]*Mathf.Rad2Deg;
                angText[i].GetComponent<TMPro.TextMeshProUGUI>().text = sliderVal[i].ToString("f2");
            }
            posText[0].GetComponent<TMPro.TextMeshProUGUI>().text = (angle[0].z * Mathf.Rad2Deg).ToString("f2");
            posText[1].GetComponent<TMPro.TextMeshProUGUI>().text = (angle[1].z * Mathf.Rad2Deg).ToString("f2");
            posText[2].GetComponent<TMPro.TextMeshProUGUI>().text = (angle[2].z * Mathf.Rad2Deg).ToString("f2");
        }
    }
}