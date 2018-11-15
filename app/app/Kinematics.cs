using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Accord.Extensions.Math;
using Accord.Math;
using AForge.Math;

namespace app
{
    public class Kinematics : INotifyPropertyChanged
    {
        Motor Spin, Shoulder, Elbow, Wrist1, Wrist2;

        public event PropertyChangedEventHandler PropertyChanged;

        public Vector3 Position { get; set; } = new Vector3();

        public Kinematics(Motor spin, Motor shoulder, Motor elbow, Motor wrist1, Motor wrist2)
        {
            Spin = spin;
            Shoulder = shoulder;
            Elbow = elbow;
            Wrist1 = wrist1;
            Wrist2 = wrist2;
        }

        public void Update()
        {
            var t1 = TransformMatrixFromDH(90, 25, Spin.Angle, 364);
            var t2 = TransformMatrixFromDH(0, 220, -Shoulder.Angle, 0);
            var t3 = TransformMatrixFromDH(0, 220, Elbow.Angle, 0);

            var result = t1.Multiply(t2).Multiply(t3);
            Position = new Vector3((float)result[0, 3], (float)result[1, 3], (float)result[2, 3]);
            this.PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("Position"));
            //var t4 = TransformMatrixFromDH(90, 220, Wrist1.Angle - Wrist2.Angle, 220);
        }

        public double[,] TransformMatrixFromDH(double alpha, double a, double theta, double d)
        {
            alpha = DegToRad(alpha);
            theta = DegToRad(theta);
            double[,] retVal = new double[4, 4];
            retVal[0, 0] = Math.Cos(theta);
            retVal[0, 1] = -Math.Sin(theta) * Math.Cos(alpha);
            retVal[0, 2] = Math.Sin(theta) * Math.Sin(alpha);
            retVal[0, 3] = a * Math.Cos(theta);

            retVal[1, 0] = Math.Sin(theta);
            retVal[1, 1] = Math.Cos(theta) * Math.Cos(alpha);
            retVal[1, 2] = -Math.Cos(theta) * Math.Sin(alpha);
            retVal[1, 3] = a * Math.Sin(theta);

            retVal[2, 0] = 0;
            retVal[2, 1] = Math.Sin(alpha);
            retVal[2, 2] = Math.Cos(alpha);
            retVal[2, 3] = d;

            retVal[3, 0] = 0;
            retVal[3, 1] = 0;
            retVal[3, 2] = 0;
            retVal[3, 3] = 1;

            return retVal;
        }

        double DegToRad(double deg)
        {
            return deg / 180 * Math.PI;
        }

        double RadToDeg(double rad)
        {
            return rad / Math.PI * 180;
        }
    }
}
