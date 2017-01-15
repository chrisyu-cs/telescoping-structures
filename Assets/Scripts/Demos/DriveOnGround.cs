using UnityEngine;
using System.Collections;

namespace Telescopes.Playground
{
    public class DriveOnGround : MonoBehaviour
    {
        bool driving = false;

        public WheelScript frontLeft;
        public WheelScript frontRight;
        public WheelScript backLeft;
        public WheelScript backRight;

        float interp = 0;

        public float levelY;

        void Start()
        {
            transform.position = Vector3.zero;
        }

        // Update is called once per frame
        void Update()
        {
            if (Input.GetKeyDown("1"))
            {
                if (!driving)
                {
                    frontLeft.StartSpinning();
                    frontRight.StartSpinning();
                    backLeft.StartSpinning();
                    backRight.StartSpinning();

                    driving = true;
                }

                else
                {
                    frontLeft.StopSpinning();
                    frontRight.StopSpinning();
                    backLeft.StopSpinning();
                    backRight.StopSpinning();

                    driving = false;
                }
            }

            if (driving)
            {
                interp = Mathf.Clamp01(interp + Time.deltaTime * 2f);
            }
            else
            {
                interp = Mathf.Clamp01(interp - Time.deltaTime * 2f);
            }
            transform.position += interp * Time.deltaTime * 5f * Vector3.forward;

            // Rotate so that wheels are horizontal
            Vector3 wheelForward = frontLeft.transform.position - backLeft.transform.position;
            wheelForward.x = 0;
            wheelForward.Normalize();
            Vector3 objForward = Vector3.forward;

            Quaternion correctiveR = Quaternion.FromToRotation(wheelForward, objForward);

            transform.rotation = correctiveR * transform.rotation;

            // Keep wheel bottoms on ground
            float minY = Mathf.Min(frontLeft.transform.position.y,
                frontRight.transform.position.y,
                backLeft.transform.position.y,
                backRight.transform.position.y);

            transform.position += (levelY - minY) * Vector3.up;
        }
    }

}