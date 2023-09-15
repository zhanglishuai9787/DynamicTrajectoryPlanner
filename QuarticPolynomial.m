function lon_qp = QuarticPolynomial(xs, vxs, axs, vxe, axe, time)
    lon_qp.b0 = xs;
    lon_qp.b1 = vxs;
    lon_qp.b2 = axs / 2.0;
    A = [3*time^2, 4*time^3
         6*time,   12*time^2];
    B = [vxe - lon_qp.b1 - 2*lon_qp.b2*time;
         axe - 2*lon_qp.b2];
    temp = A^(-1)*B;
    lon_qp.b3 = temp(1,1);
    lon_qp.b4 = temp(2,1);
end