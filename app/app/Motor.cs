using GalaSoft.MvvmLight.Command;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace app
{
    public class Motor : INotifyPropertyChanged
    {
        bool firstPositionSent = false;
        public Motor(Scorbot parent, int id)
        {
            Id = id;
            Parent = parent;
            HomeCommand = new RelayCommand(() =>
            {
                Home();
            });

            ZeroCommand = new RelayCommand(() =>
            {
                Home(false);
            });

            JogForward = new RelayCommand(() =>
            {
                Setpoint += 100;
            });

            JogBackward = new RelayCommand(() =>
            {
                Setpoint -= 100;
            });
        }

        public int Id { get; set; }
        public Scorbot Parent { get; set; }

        private int setpoint;

        public event PropertyChangedEventHandler PropertyChanged;

        public int Setpoint
        {
            get { return setpoint; }
            set {
                setpoint = value;
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("Setpoint"));
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("SetpointAngle"));
                Parent.WriteSetpoint(Id, setpoint);
            }
        }

        public RelayCommand HomeCommand { get; set; }
        public RelayCommand ZeroCommand { get; set; }
        public RelayCommand JogForward { get; set; }
        public RelayCommand JogBackward { get; set; }

        public double TicksPerDegree { get; set; }

        public double Angle
        {
            get { return position / TicksPerDegree; }
        }

        public double SetpointAngle
        {
            set { setpoint = (int)Math.Round(value * TicksPerDegree);  }
            get { return setpoint / TicksPerDegree; }
        }


        private int position;

        public int Position
        {
            get { return position; }
            internal set {
                position = value;
                if (!firstPositionSent)
                {
                    setpoint = value;
                    PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("Setpoint"));
                    PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("Angle"));
                    firstPositionSent = true;
                }
                    
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("Position"));
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("Angle"));
            }
        }

        private int minimum;

        public int Minimum
        {
            get { return minimum; }
            set { minimum = value;
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("Minimum")); }
        }

        private int maximum;

        public int Maximum
        {
            get { return maximum; }
            set
            {
                maximum = value;
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("Maximum"));
            }
        }

        public Task Home(bool withLimitSwitches = true)
        {
            setpoint = 0;
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("Setpoint"));
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs("SetpointAngle"));
            return Parent.Home(Id, withLimitSwitches);
        }

        public string Name { get; set; }

        public override string ToString()
        {
            return $"Current position: {Position}, Current setpoint: {Setpoint}";
        }
    }
}
