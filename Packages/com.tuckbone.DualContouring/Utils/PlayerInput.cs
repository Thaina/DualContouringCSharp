using JetBrains.Annotations;
using Unity.Mathematics;
using UnityEngine;

namespace Code.Utils
{
    public static class PlayerInput
    {
        enum RotationAxes { MouseXAndY = 0, MouseX = 1, MouseY = 2 }
        static RotationAxes axes = RotationAxes.MouseXAndY;
        static float2 sensitivity = 5F;

        static float minimumY = -60F;
        static float maximumY = 60F;

        static float rotationY = 0F;

        public static void Update(Transform transform,out Vector3 movementVector)
        {
            movementVector = Vector3.zero;

            if(Input.GetKey(KeyCode.W))
                movementVector += Vector3.forward;

            if(Input.GetKey(KeyCode.S))
                movementVector -= Vector3.forward;

            if(Input.GetKey(KeyCode.A))
                movementVector -= Vector3.right;

            if(Input.GetKey(KeyCode.D))
                movementVector += Vector3.right;

            if(Input.GetKey(KeyCode.Space))
                movementVector += Vector3.up;

            if(Input.GetKey(KeyCode.LeftShift))
                movementVector -= Vector3.up;

            if(Input.GetMouseButtonDown(0))
                Cursor.lockState = CursorLockMode.Locked;
            if(Input.GetKey(KeyCode.Escape))
                Cursor.lockState = CursorLockMode.None;

            if(Cursor.lockState == CursorLockMode.Locked)
            {
                if(axes == RotationAxes.MouseXAndY)
                {
                    float rotationX = transform.localEulerAngles.y + Input.GetAxis("Mouse X") * sensitivity.x;

                    rotationY += Input.GetAxis("Mouse Y") * sensitivity.y;
                    rotationY = Mathf.Clamp(rotationY,minimumY,maximumY);

                    transform.localEulerAngles = new Vector3(-rotationY,rotationX,0);
                }
                else if(axes == RotationAxes.MouseX)
                {
                    transform.Rotate(0,Input.GetAxis("Mouse X") * sensitivity.x,0);
                }
                else
                {
                    rotationY += Input.GetAxis("Mouse Y") * sensitivity.y;
                    rotationY = Mathf.Clamp(rotationY,minimumY,maximumY);

                    transform.localEulerAngles = new Vector3(-rotationY,transform.localEulerAngles.y,0);
                }
            }
        }
    }
}
