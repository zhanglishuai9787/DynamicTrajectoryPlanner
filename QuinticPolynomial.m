function lat_qp = QuinticPolynomial(xs, vxs, axs, xe, vxe, axe, time)
    lat_qp.a0 = xs;
    lat_qp.a1 = vxs;
    lat_qp.a2 = axs/2;
    A = [time^3,   time^4,     time^5;
         3*time^2, 4*time^3,   5*time^4;
         6*time,   12*time^2,  20*time^3];
    B = [xe - lat_qp.a0 - lat_qp.a1*time - lat_qp.a2*time^2;
         vxe - lat_qp.a1 - 2*lat_qp.a2*time;
         axe - 2*lat_qp.a2];
    temp = A^(-1)*B;
    lat_qp.a3 = temp(1,1);
    lat_qp.a4 = temp(2,1);
    lat_qp.a5 = temp(3,1);   
end