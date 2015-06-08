f = open('collipydata.dat','w')
# f = open('odomdata.dat','w')
f.truncate() # if u use this command, change append mode to write mode w
# np.savetxt(f,a)
f.close()

# g = np.loadtxt('myfile.dat') 
# print g