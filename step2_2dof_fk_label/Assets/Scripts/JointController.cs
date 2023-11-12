using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace InverseKinematics
{
    public class JointController : MonoBehaviour
    {
        private GameObject[] joint = new GameObject[2];
        private GameObject[] arm = new GameObject[2];
        private float[] armL = new float[2];
        private Vector3[] angle = new Vector3[2];

        private GameObject[] slider = new GameObject[2];
        private float[] sliderVal = new float[2];
        private GameObject[] argText = new GameObject[2];
        private GameObject[] posText = new GameObject[2];

        // Start is called before the first frame update
        void Start()
        {
            for(int i=0;i<joint.Length; i++){
                joint[i] = GameObject.Find("Joint_"+i.ToString());
                arm[i] = GameObject.Find("Arm_"+i.ToString());
                armL[i] = arm[i].transform.localScale.x;
            }

            for(int i=0;i<joint.Length; i++){
                slider[i] = GameObject.Find("Slider_"+i.ToString());
                argText[i] = GameObject.Find("Angle_"+i.ToString());
                sliderVal[i] = slider[i].GetComponent<Slider>().value;
            }
            posText[0] = GameObject.Find("Pos_X");
            posText[1] = GameObject.Find("Pos_Y");
        
        }

        // Update is called once per frame
        void Update()
        {
            for(int i=0;i<joint.Length;i++) {
                sliderVal[i] = slider[i].GetComponent<Slider>().value;
                argText[i].GetComponent<TMPro.TextMeshProUGUI>().text = sliderVal[i].ToString("f2");
                angle[i].z = sliderVal[i];
                joint[i].transform.localEulerAngles = angle[i];
            }
            float px = armL[0] * Mathf.Cos(angle[0].z * Mathf.Deg2Rad) 
                      + armL[1] * Mathf.Cos((angle[0].z + angle[1].z)* Mathf.Deg2Rad);
            float py = armL[0] * Mathf.Sin(angle[0].z * Mathf.Deg2Rad) 
                      + armL[1] * Mathf.Sin((angle[0].z + angle[1].z)* Mathf.Deg2Rad);
            posText[0].GetComponent<TMPro.TextMeshProUGUI>().text = px.ToString("f2");
            posText[1].GetComponent<TMPro.TextMeshProUGUI>().text = py.ToString("f2");
        }
    }
}