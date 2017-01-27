using UnityEngine;
using System.Collections.Generic;

namespace Telescopes.Playground
{
    [RequireComponent(typeof(TelescopeIK))]
    public class CraneController : MonoBehaviour
    {
        public GameObject clawGoal;

        Transform craneTarget;
        TelescopeIK ik;
        TelescopeSegment segment;

        public ClawCloser claw;

        public Vector3 targetOffset;

        public Transform bin;

        public List<Transform> targets;
        public Transform calibrationPos;

        public List<Transform> targetObjects;

        Transform currentTarget;
        Transform currentTargetObject;

        float delay = 0;
        float interp = 0;
        public CraneMode currentMode;

        int targetIndex = 0;

        // Use this for initialization
        void Start()
        {
            GameObject g = clawGoal;
            craneTarget = g.transform;

            ik = GetComponent<TelescopeIK>();
            ik.target = craneTarget;
            segment = GetComponent<TelescopeSegment>();

            craneTarget.position = calibrationPos.position;

            currentTarget = calibrationPos;
            currentMode = CraneMode.Holding;
            segment.ExtendImmediate(0);

            segment.LastShell.AlwaysExtend = true;
            segment.LastShell.SetTransform();

            Vector3 clawPos = segment.LastShell.EndPointWS;
            Vector3 clawForward = segment.LastShell.EndTangentWS;
            claw.transform.position = clawPos + clawForward * 0.1f;
            claw.transform.forward = -clawForward;

            claw.transform.parent = segment.LastShell.transform;
        }

        void FetchTarget(Transform newTarget)
        {
            currentTarget = newTarget;
            currentMode = CraneMode.Extending;
            segment.ExtendTo(1);
            delay = 0;
        }

        void DepositTarget()
        {
            currentTarget = bin;
            currentMode = CraneMode.Extending;
            segment.ExtendTo(1);
        }

        void RetrieveTarget()
        {
            currentMode = CraneMode.Retracting;
            segment.ExtendTo(0);
        }

        // Update is called once per frame
        void Update()
        {
            craneTarget.position = Vector3.Lerp(calibrationPos.position, 
                currentTarget.position + targetOffset, interp);

            switch (currentMode)
            {
                case CraneMode.Extending:
                    if (delay < 1) delay += Time.deltaTime;
                    else interp = Mathf.Clamp01(interp + Time.deltaTime);
                    if (interp >= 1) currentMode = CraneMode.Holding;
                    break;
                case CraneMode.Retracting:
                    interp = Mathf.Clamp01(interp - Time.deltaTime);
                    if (interp <= 0) currentMode = CraneMode.Holding;
                    break;
                case CraneMode.Holding:
                default:
                    break;
            }

            Vector3 clawUp = claw.transform.up;
            Vector3 clawForward = claw.transform.forward;

            Vector3 worldUpImage = Vector3.up - Vector3.Dot(Vector3.up, clawForward) * clawForward;

            float angle = TelescopeUtils.AngleBetween(clawUp, worldUpImage.normalized, claw.transform.forward);
            claw.transform.rotation = Quaternion.AngleAxis(Mathf.Min(angle, 90f * Time.deltaTime),
                claw.transform.forward) * claw.transform.rotation;

            if (Input.GetKeyDown("=") && targetIndex < targets.Count && currentMode == CraneMode.Holding)
            {
                Debug.Log("extending");
                FetchTarget(targets[targetIndex]);
                currentTargetObject = targetObjects[targetIndex];
                targetIndex++;
            }

            if (Input.GetKeyDown("-") && currentMode == CraneMode.Holding)
            {
                Debug.Log("retracting");
                RetrieveTarget();
            }

            if (Input.GetKeyDown("0") && currentMode == CraneMode.Holding)
            {
                Debug.Log("Moving to bin");
                DepositTarget();
            } 

            if (Input.GetKeyDown("1"))
            {
                bool nowOpen = claw.ToggleClose();
                if (!nowOpen)
                {
                    currentTargetObject.parent = claw.transform;
                }
                else
                {
                    currentTargetObject.parent = null;
                    currentTargetObject.GetComponent<Rigidbody>().isKinematic = false;
                }
            }
        }

        public enum CraneMode
        {
            Extending, Retracting, Holding
        }
    }


}