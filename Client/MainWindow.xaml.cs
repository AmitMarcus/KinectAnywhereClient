namespace Microsoft.Samples.Kinect.SkeletonClient
{
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using Microsoft.Kinect;
    using System.Web;
    using System.Net.Sockets;
    using System.Net;
    using System.Text;
    using System.Collections.Generic;
    using System.Reflection;
    using System;
    using System.Runtime.Serialization.Formatters.Binary;
    using System.Diagnostics;
    using System.Threading;

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        /// <summary>
        /// Width of output drawing
        /// </summary>
        private const float RenderWidth = 640.0f;

        /// <summary>
        /// Height of our output drawing
        /// </summary>
        private const float RenderHeight = 480.0f;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of body center ellipse
        /// </summary>
        private const double BodyCenterThickness = 10;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Brush used to draw skeleton center point
        /// </summary>
        private readonly Brush centerPointBrush = Brushes.Blue;

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently tracked
        /// </summary>
        private readonly Pen trackedBonePen = new Pen(Brushes.Green, 6);

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor sensor;

        /// <summary>
        /// Drawing group for skeleton rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;
        
        static Socket sock = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
        static IPAddress serverAddr = null;
        static bool isKinectON = false;
        static bool shouldExit = false;

        UdpClient Client = new UdpClient(5000);

        //CallBack
        private void recv(System.IAsyncResult res)
        {
            IPEndPoint RemoteIpEndPoint = new IPEndPoint(IPAddress.Any, 8000);
            byte[] receivedBytes = Client.EndReceive(res, ref RemoteIpEndPoint);
            string received = System.Text.Encoding.ASCII.GetString(receivedBytes);
            if (received.Contains(System.Environment.MachineName + "=OFF") && isKinectON) {
                this.sensor.Stop();
                isKinectON = false;
            }
            if (received.Contains(System.Environment.MachineName + "=ON") && !isKinectON)
            {
                this.sensor.Start();
                isKinectON = true;
            }
            if (received.Contains(System.Environment.MachineName + "=SHUTDOWN"))
            {
                var psi = new ProcessStartInfo("shutdown", "/s /t 0");
                psi.CreateNoWindow = true;
                psi.UseShellExecute = false;
                Process.Start(psi);
            }

            serverAddr = RemoteIpEndPoint.Address;

            Client.BeginReceive(new AsyncCallback(recv), null);
        }

        private void syncClock()
        {
            while (!shouldExit)
            {
                System.Diagnostics.Process process = new System.Diagnostics.Process();
                System.Diagnostics.ProcessStartInfo startInfo = new System.Diagnostics.ProcessStartInfo();
                startInfo.WindowStyle = System.Diagnostics.ProcessWindowStyle.Hidden;
                startInfo.FileName = "W32tm";
                startInfo.Arguments = "/resync /force";
                process.StartInfo = startInfo;
                process.Start();

                Thread.Sleep(1000 * 60);
            }
        }

        private void sendBeacon()
        {
            while (!shouldExit)
            {
                MemoryStream ms = new MemoryStream();

                string hostname = System.Environment.MachineName;
                byte[] hostnameBytes = System.Text.Encoding.ASCII.GetBytes(hostname);
                ms.Write(hostnameBytes, 0, hostnameBytes.Length);

                ms.WriteByte(0);

                // Write is kinect on
                if (isKinectON)
                {
                    ms.WriteByte(1);
                }
                else
                {
                    ms.WriteByte(0);
                }

                if (serverAddr != null)
                {
                    ms.Position = 0;
                    IPEndPoint endPoint = new IPEndPoint(serverAddr, 11000);
                    int ret = sock.SendTo(ms.ToArray(), endPoint);
                }

                Thread.Sleep(2000);
            }
        }

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping skeleton data
        /// </summary>
        /// <param name="skeleton">skeleton to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private static void RenderClippedEdges(Skeleton skeleton, DrawingContext drawingContext)
        {
            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, RenderHeight - ClipBoundsThickness, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, RenderHeight));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(RenderWidth - ClipBoundsThickness, 0, ClipBoundsThickness, RenderHeight));
            }
        }

        /// <summary>
        /// Execute startup tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // Display the drawing using our image control
            Image.Source = this.imageSource;

            while (this.sensor == null)
            {
                // Look through all sensors and start the first connected one.
                // This requires that a Kinect is connected at the time of app startup.
                // To make your app robust against plug/unplug, 
                // it is recommended to use KinectSensorChooser provided in Microsoft.Kinect.Toolkit (See components in Toolkit Browser).
                foreach (var potentialSensor in KinectSensor.KinectSensors)
                {
                    if (potentialSensor.Status == KinectStatus.Connected)
                    {
                        this.sensor = potentialSensor;
                        break;
                    }
                }

                if (null != this.sensor)
                {
                    // Turn on the skeleton stream to receive skeleton frames
                    this.sensor.SkeletonStream.Enable();

                    // Add an event handler to be called whenever there is new color frame data
                    this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady;

                    // Start the sensor!
                    try
                    {
                        this.sensor.Start();
                        isKinectON = true;
                    }
                    catch (IOException)
                    {
                        this.sensor = null;
                    }
                }

                Thread.Sleep(1000);
            }

            Thread beaconThread = new Thread(new ThreadStart(sendBeacon));
            beaconThread.Start();

            Thread syncClockThread = new Thread(new ThreadStart(syncClock));
            syncClockThread.Start();

            try
            {
                Client.BeginReceive(new AsyncCallback(recv), null);
            }
            catch (Exception exp)
            {
                Debug.WriteLine(exp.ToString());
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.sensor)
            {
                this.sensor.Stop();
            }

            shouldExit = true;
        }

        private void addSkeletonToMemoryStream(Skeleton skel, MemoryStream ms)
        {
            // The format of each skeleton:
            // <kineck id>
            // <skel tracking id>
            // <timestamp offset>
            // <skel joint type> <skel tracking state> <joint x,y,z> <skel joint type> <skel tracking state> <joint x,y,z> ...

            // Note: camera is expected to exist by the time this method is called.
            // The process is expected to be executed in real time, so we try to make this function call
            // as efficient as possible
            // (a low level optimization: avoid redundant if mostly not taken in each frame).
            
            // Write skel id
            byte[] skelId = BitConverter.GetBytes(skel.TrackingId);
            ms.Write(skelId, 0, skelId.Length);

            //// Write time span since session beginning, in milliseconds.
            //// This saves on some bytes since we don't have to encode the whole date format
            //// which mostly doesn't change within the session.
            //TimeSpan span = timestamp - _sessionTimestamp;
            //uint spanMs = (uint)span.TotalMilliseconds;
            //writer.Write(spanMs);

            foreach (Joint joint in skel.Joints)
            {
                byte jointType = (byte)joint.JointType; // Enum
                byte jointTrackingState = (byte)joint.TrackingState; // Enum
                byte[] jointTypebyte = new byte[1];
                jointTypebyte[0] = jointType;
                byte[] jointTrackingStatebyte = new byte[1];
                jointTrackingStatebyte[0] = jointTrackingState;
                byte[] jointXbyte = BitConverter.GetBytes(joint.Position.X);
                byte[] jointYbyte = BitConverter.GetBytes(joint.Position.Y);
                byte[] jointZbyte = BitConverter.GetBytes(joint.Position.Z);

                ms.Write(jointTypebyte, 0, jointTypebyte.Length);
                ms.Write(jointTrackingStatebyte, 0, jointTrackingStatebyte.Length);
                ms.Write(jointXbyte, 0, jointXbyte.Length);
                ms.Write(jointYbyte, 0, jointYbyte.Length);
                ms.Write(jointZbyte, 0, jointZbyte.Length);
            }

            byte[] endOfSkel = new byte[2] { 0xFF, 0xFF };
            ms.Write(endOfSkel, 0, endOfSkel.Length);
        }


        /// <summary>
        /// Event handler for Kinect sensor's SkeletonFrameReady event
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] skeletons = new Skeleton[0];

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);
                }
            }

            using (DrawingContext dc = this.drawingGroup.Open())
            {
                // Draw a transparent background to set the render size
                dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));

                if (skeletons.Length != 0)
                {
                    MemoryStream ms = new MemoryStream();

                    // Write hostname
                    string hostname = System.Environment.MachineName;
                    byte[] hostnameBytes = System.Text.Encoding.ASCII.GetBytes(hostname);
                    ms.Write(hostnameBytes, 0, hostnameBytes.Length);

                    byte[] nullByte = new byte[1]; // null terminator for hostname
                    nullByte[0] = 0;
                    ms.Write(nullByte, 0, nullByte.Length);

                    // Write is kinect on
                    if (isKinectON)
                    {
                        ms.WriteByte(1);
                    }
                    else
                    {
                        ms.WriteByte(0);
                    }

                    // Write timestamp
                    double timestamp = DateTime.Now.ToUniversalTime().Subtract(new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc)).TotalMilliseconds;
                    byte[] timestampBytes = BitConverter.GetBytes(timestamp);
                    ms.Write(timestampBytes, 0, timestampBytes.Length);
                    
                    // Write skeletons
                    foreach (Skeleton skel in skeletons)
                    {
                        RenderClippedEdges(skel, dc);

                        if (skel.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            // Draws the skeleton
                            this.DrawBonesAndJoints(skel, dc);

                            addSkeletonToMemoryStream(skel, ms);

                        }
                        else if (skel.TrackingState == SkeletonTrackingState.PositionOnly)
                        {
                            dc.DrawEllipse(
                            this.centerPointBrush,
                            null,
                            this.SkeletonPointToScreen(skel.Position),
                            BodyCenterThickness,
                            BodyCenterThickness);
                        }
                    }


                    if (serverAddr != null && ms.Length > timestampBytes.Length + hostnameBytes.Length + hostnameBytes.Length + nullByte.Length + 1)
                    {
                        ms.Position = 0;
                        IPEndPoint endPoint = new IPEndPoint(serverAddr, 11000);
                        int ret = sock.SendTo(ms.ToArray(), endPoint);
                    }
                }

                // prevent drawing outside of our render area
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
            }
        }

        /// <summary>
        /// Draws a skeleton's bones and joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawBonesAndJoints(Skeleton skeleton, DrawingContext drawingContext)
        {
            // Render Torso
            this.DrawBone(skeleton, drawingContext, JointType.Head, JointType.ShoulderCenter);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderRight);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.Spine);
            this.DrawBone(skeleton, drawingContext, JointType.Spine, JointType.HipCenter);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipLeft);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipRight);

            // Left Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderLeft, JointType.ElbowLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowLeft, JointType.WristLeft);
            this.DrawBone(skeleton, drawingContext, JointType.WristLeft, JointType.HandLeft);

            // Right Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderRight, JointType.ElbowRight);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowRight, JointType.WristRight);
            this.DrawBone(skeleton, drawingContext, JointType.WristRight, JointType.HandRight);

            // Left Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipLeft, JointType.KneeLeft);
            this.DrawBone(skeleton, drawingContext, JointType.KneeLeft, JointType.AnkleLeft);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleLeft, JointType.FootLeft);

            // Right Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipRight, JointType.KneeRight);
            this.DrawBone(skeleton, drawingContext, JointType.KneeRight, JointType.AnkleRight);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleRight, JointType.FootRight);
 
            // Render Joints
            foreach (Joint joint in skeleton.Joints)
            {
                Brush drawBrush = null;

                if (joint.TrackingState == JointTrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;                    
                }
                else if (joint.TrackingState == JointTrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;                    
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, this.SkeletonPointToScreen(joint.Position), JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Maps a SkeletonPoint to lie within our render space and converts to Point
        /// </summary>
        /// <param name="skelpoint">point to map</param>
        /// <returns>mapped point</returns>
        private Point SkeletonPointToScreen(SkeletonPoint skelpoint)
        {
            // Convert point to depth space.  
            // We are not using depth directly, but we do want the points in our 640x480 output resolution.
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            return new Point(depthPoint.X, depthPoint.Y);
        }

        /// <summary>
        /// Draws a bone line between two joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw bones from</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="jointType0">joint to start drawing from</param>
        /// <param name="jointType1">joint to end drawing at</param>
        private void DrawBone(Skeleton skeleton, DrawingContext drawingContext, JointType jointType0, JointType jointType1)
        {
            Joint joint0 = skeleton.Joints[jointType0];
            Joint joint1 = skeleton.Joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == JointTrackingState.NotTracked ||
                joint1.TrackingState == JointTrackingState.NotTracked)
            {
                return;
            }

            // Don't draw if both points are inferred
            if (joint0.TrackingState == JointTrackingState.Inferred &&
                joint1.TrackingState == JointTrackingState.Inferred)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if (joint0.TrackingState == JointTrackingState.Tracked && joint1.TrackingState == JointTrackingState.Tracked)
            {
                drawPen = this.trackedBonePen;
            }

            drawingContext.DrawLine(drawPen, this.SkeletonPointToScreen(joint0.Position), this.SkeletonPointToScreen(joint1.Position));
        }

        /// <summary>
        /// Handles the checking or unchecking of the seated mode combo box
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void CheckBoxSeatedModeChanged(object sender, RoutedEventArgs e)
        {
            if (null != this.sensor)
            {
                if (this.checkBoxSeatedMode.IsChecked.GetValueOrDefault())
                {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
                }
                else
                {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Default;
                }
            }
        }
    }
}