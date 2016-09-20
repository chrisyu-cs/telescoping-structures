using UnityEngine;
using System.Collections;

namespace Telescopes
{
    public class CameraControl : MonoBehaviour
    {
        bool locked;
        private float theta, phi;
        public float translateSpeed = 2f;
        public float rotateSpeed = 2f;

        private Rigidbody rb;
        
        // Use this for initialization
        void Start()
        {
            locked = false;
            theta = 0;
            phi = 0;
            rb = GetComponent<Rigidbody>();
        }

        void ProcessMove()
        {
            float up = Input.GetAxis("Vertical");
            float right = Input.GetAxis("Horizontal");
            float rmb = Input.GetAxis("Jump");

            transform.position += up * transform.forward * Time.deltaTime * translateSpeed;
            transform.position += right * transform.right * Time.deltaTime * translateSpeed;
            transform.position += rmb * transform.up * Time.deltaTime * translateSpeed;

            if (!locked && Input.GetAxis("Fire2") > 0.5f)
            {
                float mouseX = Input.GetAxis("Mouse X");
                float mouseY = -Input.GetAxis("Mouse Y");

                theta = Mathf.Repeat(theta + rotateSpeed * mouseX, 360);
                phi = Mathf.Clamp(phi + rotateSpeed * mouseY, -90, 90);
            }

            transform.rotation = Quaternion.Euler(phi, theta, 0);
        }

        // Update is called once per frame
        void Update()
        {
            ProcessMove();
        }
    }
}