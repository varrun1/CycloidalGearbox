import matplotlib.pyplot as plt
import matplotlib.patches as ptc
import numpy as np

# Constants
fullROT = 360.0 #degrees
timeDuration = 20.0 #seconds
secInMin = 60.0 #s
reduction = 29.0

#Raw Input & Output Data
inputRPM = np.array([100,200,300,400,500])
output_revs = np.array([1+60/fullROT, 2+120/fullROT,3+150/fullROT,4.583,5+270/fullROT]) # output disc revolutions
time_actual = np.array([19.842,19.842, 19.842,19.842, 19.736])

#Compute output theoretical and measured output RPM 
#print(output_revs)
outputRPM_actual = (output_revs/time_actual)*secInMin
outputRPM_act_adj = (output_revs/timeDuration)*secInMin
outputRPM_theoretical = inputRPM/reduction
print('Theoretical output:',outputRPM_theoretical)
print('Adjusted Actual Output:',outputRPM_act_adj)
print('Actual Output:',outputRPM_actual)

#Compute residuals
reference = outputRPM_theoretical
residual1 = outputRPM_act_adj - reference
residual2 = outputRPM_actual - reference

#Compute %
percentDiff = ((outputRPM_act_adj-outputRPM_theoretical)/outputRPM_theoretical)*100.0
print('% Difference:',percentDiff)
mape = np.mean(np.abs((outputRPM_act_adj-outputRPM_theoretical)/outputRPM_theoretical)) * 100.0
print('MAPE (%):',mape)

#Plotting output RPM vs input RPM
plt.figure(figsize=(8,5))
plt.subplot(2,1,1)
plt.plot(inputRPM,outputRPM_theoretical, marker='o', linestyle='-', markersize=5, label='Theoretical Output RPM')
plt.plot(inputRPM,outputRPM_act_adj, marker='o', linestyle='-', markersize=5, color = 'green', label='Adjusted Measured Output')
plt.plot(inputRPM,outputRPM_actual, marker='o', linestyle='--', markersize=5,color = 'purple', label='Measured Output')
plt.legend(bbox_to_anchor=(1.02,1), loc='upper left', ncol=1)
plt.xlabel('Input RPM')
plt.ylabel('Output RPM')
plt.title('Reduction Ratio Testing')
plt.grid(True)
plt.tight_layout()
#plt.show()

#Plotting input RPM vs output RPM residuals
plt.subplot(2,1,2)
plt.plot(inputRPM,residual1, marker='o', linestyle='-', markersize=5, color = 'green', label='Adjusted Measured Residual')
plt.plot(inputRPM,residual2, marker='o', linestyle='-', markersize=5, color = 'purple', label='Measured Output Residual')
plt.legend(bbox_to_anchor=(1.02,1), loc='upper left', ncol=1)
plt.axhline(0, linestyle="--", linewidth=1, color = 'red')
plt.xlabel('Input RPM')
plt.ylabel('Residual')
plt.title('Reduction Ratio Testing Residuals')
plt.ylim(-0.30, 0.30)
plt.grid(True)
plt.tight_layout()
plt.show()



