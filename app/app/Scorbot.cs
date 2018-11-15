using Charlotte;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace app
{
    public class Scorbot : MqttModule
    {
        public ObservableCollection<Motor> Motors { get; set; } = new ObservableCollection<Motor>();
        public Kinematics Kinematics { get; set; }
        public Scorbot() : base("192.168.0.1", 1883)
        {
            for (int i = 0; i < 7; i++)
                Motors.Add(new Motor(this, i));

            Motors[0].Name = "Base";
            Motors[0].TicksPerDegree = 6500 / 90;
            Motors[0].Minimum = -11000;
            Motors[0].Maximum = 9000;

            Motors[1].Name = "Shoulder";
            Motors[1].TicksPerDegree = 20000/360;
            Motors[1].Minimum = -5000;
            Motors[1].Maximum = 5000;

            Motors[2].Name = "Elbow";
            Motors[2].TicksPerDegree = 20000/360;
            Motors[2].Minimum = -10000;
            Motors[2].Maximum = 10000;

            Motors[3].Name = "Wrist #1";
            Motors[3].Minimum = -10000;
            Motors[3].Maximum = 10000;

            Motors[4].Name = "Wrist #2";
            Motors[4].Minimum = -10000;
            Motors[4].Maximum = 10000;

            Motors[5].Name = "Gripper";
            Motors[5].Minimum = 0;
            Motors[5].Maximum = 2500;

            Motors[6].Name = "Slide";
            Motors[6].Minimum = 0;
            Motors[6].Maximum = 4400;

            Kinematics = new Kinematics(Motors[0], Motors[1], Motors[2], Motors[3], Motors[4]);

            On["motors/position/{id}"] = msg =>
            {
                Motors[Int32.Parse(msg.id)].Position = Int32.Parse(msg.Message);
                Kinematics.Update();
            };

            this.Connect();

        }
        internal Task WriteSetpoint(int id, int setpoint)
        {
            return Task.Run(() =>
            {
                this.Publish($"motors/setpoint/{id}", setpoint.ToString());
            });
        }

        internal Task Home(int id, bool withLimitSwitches = true)
        {
            return Task.Run(() =>
            {
                this.Publish($"motors/home/{id}", withLimitSwitches ? "1" : "0");
            });
        }

        internal void Close()
        {
            this.Disconnect();
        }
    }
}
