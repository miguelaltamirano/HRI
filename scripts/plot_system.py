i=0
for i in range(500):
    #Plot trajectory
        clear_output(wait=True)
        fig = plt.figure()
        ax = fig.add_subplot(3, 1, 1)
        
        if i>40:
            ax.plot(Pp[0,(i-40):i+1], Pp[1,(i-40):i+1], 'r')  #plot 4o last points   
        else:
            ax.plot(Pp[0,0:i+1], Pp[1,0:i+1], 'r')  #plot 4o last points
        
        ax.plot(Pp[0,i], Pp[1,i], 'ro')   #plot last position
        ax.plot(0.15, y_refr, 'bs',0.9, y_refr, 'bs') #plot target points
        ax.plot(xo_refr,yo_refr,'rs')  #plot obstacle
        circle1 =plt.Circle((xo_refr,yo_refr), dmin, color='g', fill=False)  #plot inverse potential field
        circle2 =plt.Circle((xo_refr,yo_refr), dmin2, color='y', fill=False)  #plot inverse potential field
        ax.add_artist(circle1)
        ax.add_artist(circle2)
        ax.axis([0.1, 1, -0.55, -0.3])
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.grid(which='both')
        
        ax2 = fig.add_subplot(3, 1, 2)
        ax2.plot(Pp[0,:i+1], 'k-')   #plot last position
        ax2.plot(i,Pp[0,i], 'ko')   #plot last position
        ax2.grid(which='both')
        
        ax3 = fig.add_subplot(3, 1, 3)
        ax3.plot(Pp[1,:i+1], 'k-')   #plot last position
        ax3.plot(i,Pp[1,i], 'ko')   #plot last position
        ax3.grid(which='both')
        plt.show()