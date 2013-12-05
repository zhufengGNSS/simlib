
set style data lines

set terminal postscript; set output "silnikov2.ps"
plot "silnikov2.dat" 

set terminal postscript; set output "silnikov2a.ps"
splot "silnikov2.dat" 

set terminal postscript; set output "silnikov2b.ps"
plot "silnikov2.dat"  using 2:3 


