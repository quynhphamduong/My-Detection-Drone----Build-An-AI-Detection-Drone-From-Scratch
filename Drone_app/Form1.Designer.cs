namespace Drone_app
{
    partial class Form1
    {
        /// <summary>
        ///  Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        ///  Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        ///  Required method for Designer support - do not modify
        ///  the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(Form1));
            panelMenu = new Panel();
            aboutButton = new FontAwesome.Sharp.IconButton();
            settingsButton = new FontAwesome.Sharp.IconButton();
            navigationButton = new FontAwesome.Sharp.IconButton();
            flightControlButton = new FontAwesome.Sharp.IconButton();
            dashboardButton = new FontAwesome.Sharp.IconButton();
            panelLogo = new Panel();
            pictureBox1 = new PictureBox();
            panel1 = new Panel();
            panelMenu.SuspendLayout();
            panelLogo.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)pictureBox1).BeginInit();
            SuspendLayout();
            // 
            // panelMenu
            // 
            panelMenu.BackColor = Color.FromArgb(31, 30, 68);
            panelMenu.Controls.Add(aboutButton);
            panelMenu.Controls.Add(settingsButton);
            panelMenu.Controls.Add(navigationButton);
            panelMenu.Controls.Add(flightControlButton);
            panelMenu.Controls.Add(dashboardButton);
            panelMenu.Controls.Add(panelLogo);
            panelMenu.Dock = DockStyle.Left;
            panelMenu.Location = new Point(0, 0);
            panelMenu.Name = "panelMenu";
            panelMenu.Size = new Size(200, 522);
            panelMenu.TabIndex = 1;
            // 
            // aboutButton
            // 
            aboutButton.Dock = DockStyle.Top;
            aboutButton.FlatAppearance.BorderSize = 0;
            aboutButton.FlatStyle = FlatStyle.Flat;
            aboutButton.ForeColor = SystemColors.InactiveCaption;
            aboutButton.IconChar = FontAwesome.Sharp.IconChar.BookAtlas;
            aboutButton.IconColor = Color.Gainsboro;
            aboutButton.IconFont = FontAwesome.Sharp.IconFont.Auto;
            aboutButton.ImageAlign = ContentAlignment.MiddleLeft;
            aboutButton.Location = new Point(0, 380);
            aboutButton.Name = "aboutButton";
            aboutButton.Size = new Size(200, 60);
            aboutButton.TabIndex = 5;
            aboutButton.Text = "About";
            aboutButton.UseVisualStyleBackColor = true;
            aboutButton.Click += aboutButton_Click;
            // 
            // settingsButton
            // 
            settingsButton.Dock = DockStyle.Top;
            settingsButton.FlatAppearance.BorderSize = 0;
            settingsButton.FlatStyle = FlatStyle.Flat;
            settingsButton.ForeColor = SystemColors.InactiveCaption;
            settingsButton.IconChar = FontAwesome.Sharp.IconChar.Sliders;
            settingsButton.IconColor = Color.Gainsboro;
            settingsButton.IconFont = FontAwesome.Sharp.IconFont.Auto;
            settingsButton.ImageAlign = ContentAlignment.MiddleLeft;
            settingsButton.Location = new Point(0, 320);
            settingsButton.Name = "settingsButton";
            settingsButton.Size = new Size(200, 60);
            settingsButton.TabIndex = 4;
            settingsButton.Text = "Settings";
            settingsButton.UseVisualStyleBackColor = true;
            settingsButton.Click += settingsButton_Click;
            // 
            // navigationButton
            // 
            navigationButton.Dock = DockStyle.Top;
            navigationButton.FlatAppearance.BorderSize = 0;
            navigationButton.FlatStyle = FlatStyle.Flat;
            navigationButton.ForeColor = SystemColors.InactiveCaption;
            navigationButton.IconChar = FontAwesome.Sharp.IconChar.MapLocationDot;
            navigationButton.IconColor = Color.Gainsboro;
            navigationButton.IconFont = FontAwesome.Sharp.IconFont.Auto;
            navigationButton.ImageAlign = ContentAlignment.MiddleLeft;
            navigationButton.Location = new Point(0, 260);
            navigationButton.Name = "navigationButton";
            navigationButton.Size = new Size(200, 60);
            navigationButton.TabIndex = 3;
            navigationButton.Text = "Navigation";
            navigationButton.UseVisualStyleBackColor = true;
            navigationButton.Click += navigationButton_Click;
            // 
            // flightControlButton
            // 
            flightControlButton.Dock = DockStyle.Top;
            flightControlButton.FlatAppearance.BorderSize = 0;
            flightControlButton.FlatStyle = FlatStyle.Flat;
            flightControlButton.ForeColor = SystemColors.InactiveCaption;
            flightControlButton.IconChar = FontAwesome.Sharp.IconChar.Gamepad;
            flightControlButton.IconColor = Color.Gainsboro;
            flightControlButton.IconFont = FontAwesome.Sharp.IconFont.Auto;
            flightControlButton.ImageAlign = ContentAlignment.MiddleLeft;
            flightControlButton.Location = new Point(0, 200);
            flightControlButton.Name = "flightControlButton";
            flightControlButton.Size = new Size(200, 60);
            flightControlButton.TabIndex = 2;
            flightControlButton.Text = "Flight Control";
            flightControlButton.UseVisualStyleBackColor = true;
            flightControlButton.Click += flightControlButton_Click;
            // 
            // dashboardButton
            // 
            dashboardButton.Dock = DockStyle.Top;
            dashboardButton.FlatAppearance.BorderSize = 0;
            dashboardButton.FlatStyle = FlatStyle.Flat;
            dashboardButton.ForeColor = SystemColors.InactiveCaption;
            dashboardButton.IconChar = FontAwesome.Sharp.IconChar.Delicious;
            dashboardButton.IconColor = Color.Gainsboro;
            dashboardButton.IconFont = FontAwesome.Sharp.IconFont.Auto;
            dashboardButton.ImageAlign = ContentAlignment.MiddleLeft;
            dashboardButton.Location = new Point(0, 140);
            dashboardButton.Name = "dashboardButton";
            dashboardButton.Size = new Size(200, 60);
            dashboardButton.TabIndex = 1;
            dashboardButton.Text = "Dashboard";
            dashboardButton.UseVisualStyleBackColor = true;
            dashboardButton.Click += dashboardButton_Click;
            // 
            // panelLogo
            // 
            panelLogo.Controls.Add(pictureBox1);
            panelLogo.Dock = DockStyle.Top;
            panelLogo.Location = new Point(0, 0);
            panelLogo.Name = "panelLogo";
            panelLogo.Size = new Size(200, 140);
            panelLogo.TabIndex = 0;
            // 
            // pictureBox1
            // 
            pictureBox1.Image = (Image)resources.GetObject("pictureBox1.Image");
            pictureBox1.Location = new Point(96, 6);
            pictureBox1.Name = "pictureBox1";
            pictureBox1.Size = new Size(101, 134);
            pictureBox1.SizeMode = PictureBoxSizeMode.StretchImage;
            pictureBox1.TabIndex = 0;
            pictureBox1.TabStop = false;
            // 
            // panel1
            // 
            panel1.BackColor = Color.FromArgb(26, 25, 62);
            panel1.Dock = DockStyle.Top;
            panel1.Location = new Point(200, 0);
            panel1.Name = "panel1";
            panel1.Size = new Size(693, 75);
            panel1.TabIndex = 2;
            // 
            // Form1
            // 
            AutoScaleDimensions = new SizeF(8F, 20F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new Size(893, 522);
            Controls.Add(panel1);
            Controls.Add(panelMenu);
            ForeColor = SystemColors.AppWorkspace;
            KeyPreview = true;
            Name = "Form1";
            Text = "Form1";
            panelMenu.ResumeLayout(false);
            panelLogo.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)pictureBox1).EndInit();
            ResumeLayout(false);
        }

        #endregion
        private Panel panelMenu;
        private Panel panelLogo;
        private FontAwesome.Sharp.IconButton dashboardButton;
        private FontAwesome.Sharp.IconButton aboutButton;
        private FontAwesome.Sharp.IconButton settingsButton;
        private FontAwesome.Sharp.IconButton navigationButton;
        private FontAwesome.Sharp.IconButton flightControlButton;
        private PictureBox pictureBox1;
        private Panel panel1;
    }
}
