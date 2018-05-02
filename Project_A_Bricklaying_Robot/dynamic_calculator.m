syms l1 l2 l3 l4 m g w0 dw0 dv0 c1 s1 c2 s2 c3 s3 s4 c4 dq dq1 dq2 dq3 dq4 dw Pc F N f n m1 m2 m3 m4 ddq1 ddq2 ddq3 ddq4 G
R=cell(1,6);w=cell(1,6);dw=cell(1,6);dq=cell(1,6);ddq=cell(1,6);dv=cell(1,6); dvc=cell(1,6); f=cell(1,6);n=cell(1,6);
P=cell(1,6);Pc=cell(1,6);F=cell(1,6);m=cell(1,6);N=cell(1,6);I=cell(1,6);
%initialize
R{1}=[c1 -s1 0;
      s1 c1 0;
      0 0 1];
R{2}=[-s2 -c2 0;
      0 0 -1;
      c2 -s2 1];
R{3}=[s3 c3 0;
      -c3 s3 0;
       0  0  0];
R{4}=[c4 -s4 0;
      s4 c4 0;
      0 0 1];
R{5}=[1 0 0;
      0 0 1;
      0 -1 0];
dv{1}=[0;0;g];
P{1}=[0;0;l1];P{2}=[0;0;0];P{3}=[l2;0;0];P{4}=[l3;0;0];P{5}=[0;-l4;0];
Pc{2}=[0;0;0];Pc{3}=[l2;0;0];Pc{4}=[l3;0;0];Pc{5}=[0;-l4;0];
m{2}=m1;m{3}=m2;m{4}=m3;m{5}=m4;
w{1}=[0;0;0];
dw{1}=[0;0;0];
dq{2}=[0;0;dq1];dq{3}=[0;0;dq2];dq{4}=[0;0;dq3];dq{5}=[0;0;dq4];
ddq{2}=[0;0;ddq1];ddq{3}=[0;0;ddq2];ddq{4}=[0;0;ddq3];ddq{5}=[0;0;ddq4];
f{6}=[0;0;G];
n{6}=[0;0;0];
%outward iterations
for i=1:4
w{i+1}=R{i}.'*w{i}+dq{i+1};
dw{i+1}=R{i}.'*dw{i}+cross(R{i}.'*w{i},dq{i+1})+ddq{i+1};
dv{i+1}=R{i}.'*(cross(dw{i},P{i})+cross(w{i},cross(w{i},P{i}))+dv{i});
dvc{i+1}=cross(dw{i+1},Pc{i+1})+cross(w{i+1},cross(w{i+1},Pc{i+1}))+dv{i+1};
F{i+1}=m{i+1}*dvc{i+1};
N{i+1}=[0;0;0];
end
%inward iterations
for i=5:-1:2
    f{i}=R{i}*f{i+1}+F{i};
    n{i}=N{i}+R{i}*n{i+1}+cross(Pc{i},F{i})+cross(P{i},R{i}*f{i+1});
end
tau4 = simplify(n{5}.'*[0;0;1])
tau3 = simplify(n{4}.'*[0;0;1])
tau2 = simplify(n{3}.'*[0;0;1])
tau1 = simplify(n{2}.'*[0;0;1])