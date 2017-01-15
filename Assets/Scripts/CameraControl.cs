using UnityEngine;
using System.Collections;

namespace Telescopes
{
    public class CameraControl : MonoBehaviour
    {
        bool locked;
        public float theta, phi;
        public float translateSpeed = 2f;
        public float rotateSpeed = 2f;

        public ControlMode mode;

        public float InterpRate = 0.2f;
        public float LookRadius;
        public Transform LookTarget;

        private int screenshotNum = 0;

        public bool Freeze = false;
        public bool CinematicMode = false;

        float interpUp, interpRight, interpJump;

        Camera thisCamera;
        public Camera alternateCamera;

        // Use this for initialization
        void Start()
        {
            locked = false;
            theta = transform.rotation.eulerAngles.y;
            phi = transform.rotation.eulerAngles.x;
            
            thisCamera = GetComponent<Camera>();
            if (alternateCamera) alternateCamera.enabled = false;
        }

        void ProcessTranslate()
        {
            float up = Input.GetAxis("Vertical");
            float right = Input.GetAxis("Horizontal");
            float rmb = Input.GetAxis("Jump");

            if (CinematicMode)
            {
                interpUp = Mathf.Lerp(interpUp, up, 0.05f);
                interpRight = Mathf.Lerp(interpRight, right, 0.05f);
                interpJump = Mathf.Lerp(interpJump, rmb, 0.05f);
            }
            else
            {
                interpUp = up;
                interpRight = right;
                interpJump = rmb;
            }

            Vector3 pos = transform.position;

            pos += interpUp * transform.forward * Time.deltaTime * translateSpeed;
            pos += interpRight * transform.right * Time.deltaTime * translateSpeed;
            pos += interpJump * transform.up * Time.deltaTime * translateSpeed;

            transform.position = pos;
        }

        void ProcessFreeMove()
        {
            ProcessTranslate();

            if (!locked && Input.GetAxis("Fire2") > 0.5f)
            {
                float mouseX = Input.GetAxis("Mouse X");
                float mouseY = -Input.GetAxis("Mouse Y");

                theta = Mathf.Repeat(theta + rotateSpeed * mouseX, 360);
                phi = Mathf.Clamp(phi + rotateSpeed * mouseY, -90, 90);
            }

            Quaternion q = Quaternion.Euler(phi, theta, 0);
            if (CinematicMode) transform.rotation = Quaternion.Slerp(transform.rotation, q, 0.1f);
            else transform.rotation = q;
        }

        void ProcessLookAtTarget()
        {
            Vector3 toTarget = LookTarget.position - transform.position;

            Vector3 lookPosition = LookTarget.position - (toTarget.normalized * LookRadius);
            Quaternion lookRotation = Quaternion.LookRotation(toTarget, Vector3.up);
            transform.position = Vector3.Lerp(transform.position, lookPosition, InterpRate);
            transform.rotation = Quaternion.Slerp(transform.rotation, lookRotation, InterpRate);

            phi = transform.rotation.eulerAngles.x;
            theta = transform.rotation.eulerAngles.y;
        }

        public void FreeMove()
        {
            phi =  transform.rotation.eulerAngles.x;
            if (phi > 180) phi -= 360;
            theta = transform.rotation.eulerAngles.y;

            mode = ControlMode.FreeMove;
        }

        public void LookAtTarget(Transform target, float radius)
        {
            LookTarget = target;
            LookRadius = radius;
            mode = ControlMode.LookAtTarget;
        }

        // Update is called once per frame
        void Update()
        {
            if (mode == ControlMode.FreeMove)
            {
                ProcessFreeMove();
            }
            else if (LookTarget && mode == ControlMode.LookAtTarget)
            {
                ProcessLookAtTarget();
            }
            else if (LookTarget && mode == ControlMode.TrackTarget)
            {
                if (!Freeze) ProcessTranslate();
                Quaternion q = Quaternion.LookRotation(LookTarget.position - transform.position);
                transform.rotation = Quaternion.Slerp(transform.rotation, q, 0.1f);
            }

            if (Input.GetKeyDown("f12"))
            {
                Application.CaptureScreenshot("Pictures/screenshot" + screenshotNum + ".png");
                screenshotNum++;
                Debug.Log("Screenshot saved");
            }

            if (alternateCamera && Input.GetKeyDown("f1"))
            {
                if (thisCamera.enabled)
                {
                    alternateCamera.enabled = true;
                    thisCamera.enabled = false;
                }
                else
                {
                    thisCamera.enabled = true;
                    alternateCamera.enabled = false;
                }
            }
        }

        public enum ControlMode
        {
            FreeMove, LookAtTarget, TrackTarget
        }
    }
}