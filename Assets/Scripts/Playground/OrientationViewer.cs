using UnityEngine;
using System.Collections;

namespace Telescopes
{
    public class OrientationViewer : MonoBehaviour
    {
        public void SetOrientation(OrthonormalFrame frame)
        {
            transform.rotation = Quaternion.LookRotation(frame.T, frame.B);
        }

        // Use this for initialization
        void Start()
        {

        }

        // Update is called once per frame
        void Update()
        {

        }
    }

}