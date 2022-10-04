# CircularMotion

Flight tests of the path following vector field method on the formation of Crazyflie 2.1 nano quadcopters.
The Lighthouse positioning system is used for experiments. https://www.bitcraze.io/documentation/lighthouse/

(CX, CY) (or $\(c_e, c_n\)$ ) $-$ circle center coordinates \
$k$ $-$ smoothness coefficient \
$R$ (or $\rho$) $-$ circle radius \
$v$ $-$ Crazyflie speed \
$d$ $-$ distance from Crazyflie to a circle center \
phi (or $\varphi$) $-$ phase angle \
angle (or $\chi^c$) $-$ command course angle of Crazyflie \
vx (or $v_x$) $-$ speed of Crazyflie along the x-axis in the world (global) coordinate system \
vy (or $v_y$) $-$ speed of Crazyflie along the y-axis in the world (global) coordinate system \
vz (or $v_z$) $-$ speed of Crazyflie along the z-axis in the world (global) coordinate system \
px (or $p_e$) $-$ coordinate of Crazyflie along the x-axis in the world (global) coordinate system \
py (or $p_n$) $-$ coordinate of Crazyflie along the y-axis in the world (global) coordinate system \
stateEstimate.x $-$ estimation of the copter's position with the Kalman filter along the x-axis in the world (global) coordinate system \
stateEstimate.y $-$ estimation of the copter's position with the Kalman filter along the y-axis in the world (global) coordinate system 

$\textbf{Control law for the Crazyflie course angle:}$

$\chi^c=\varphi +\lambda(\frac{\pi }{2}+\mathrm{atan}(k(d-\rho )))$, 
where $\lambda=1$ means clockwise motion and $\lambda=-1$ means counterclockwise motion.

$\textbf{Control law for the Crazyflie speeds:}$ 
[Crazyflie speed commands](https://github.com/TagirMuslimov/CircularMotion/files/9704951/document.pdf)

<figure>
  <img
  src="https://user-images.githubusercontent.com/81864311/192131806-55c5d791-378b-4b18-b842-3c1b783209b4.jpg" width="500"
  alt="2022-09-25 11_48_16-Lighthouse Positioning System _ Bitcraze">
    <figcaption><b><sub><sup>Lighthouse Positioning System</sub></sup></b></figcaption>
</figure> 
$$\\[2in]$$

<figure>
  <img
  src="https://user-images.githubusercontent.com/81864311/191765733-3b06767f-8ce0-4663-aaa3-fac17b1041f2.jpg" width="500"
  alt="2022-09-22 18_52_36-beard_r_w_mclain_t_w_small_unmanned_aircraft_theory_and_prac pdf - SumatraPDF">
    <figcaption><b><sub><sup>Notation used. Picture from [1]</sub></sup></b></figcaption>
</figure> 
$$\\[2in]$$
References <br />
[1] Beard, R. W., & McLain, T. W. (2012). Small unmanned aircraft: Theory and practice. Princeton university press.
