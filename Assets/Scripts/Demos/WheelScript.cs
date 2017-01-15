using UnityEngine;
using System.Collections;

namespace Telescopes.Playground
{
    public class WheelScript : MonoBehaviour
    {
        public Transform spinner;
        bool spinning = false;
        public bool reverse;

        // Use this for initialization
        void Start()
        {
            Debug.Log(spinner);
        }

        public void StartSpinning()
        {
            spinning = true;
        }

        public void StopSpinning()
        {
            spinning = false;
        }

        void Update()
        {
            if (spinning)
            {
                Quaternion rot = (reverse) ?
                    Quaternion.AngleAxis(-Time.deltaTime * 360, Vector3.right) :
                    Quaternion.AngleAxis(Time.deltaTime * 360, Vector3.right);
                spinner.transform.rotation *= rot;
            }
        }
    }

}