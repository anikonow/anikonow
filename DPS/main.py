import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np



df = pd.read_excel("DPS\dpslightfall.xlsx")
df = df.fillna(0)
#print(df)


time = np.arange(0,60.1,.1) #df["Time"]

damcol = ["Apex w/bipod", "Apex recon B&S", "Apex recon Surr", "Darci", "malfesance", "Briars Contempt",
    "Legend of Acrius", "Deliverance rocket", "Grapple Rocket", "ghorn", "Bastion+Windie",
    "Wini+FC+Clown", "Rocket+FP", "Star+izi+Rocket", "Star Rocket", "D Slug", "Sleeper",
    "IZ+TripRocket", "Izi+Rocket", "Clown Rocket", "Wither+Cata", "Wisper", "Izi Windy",
    "Demo +RF Gl", "Starfire+Typon"
]

for col in damcol:
    cumulative = df[col].fillna(0).cumsum()
    sns.lineplot(x=time,y=cumulative,label=col)

plt.title("Total Damage vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Total Damage")
plt.legend(title="Loadout",loc='upper right',fontsize='small')
plt.xlim(0,35)
plt.ylim(0,1900000)
plt.tight_layout()
plt.show()


for col in damcol:
    cumulative = df[col].fillna(0).cumsum()
    dps = cumulative/time
    sns.lineplot(x=time,y=dps,label=col)


plt.title("DPS vs Time")
plt.xlabel("Time (s)")
plt.ylabel("DPS")
plt.legend(title="Loadout",loc='upper right',fontsize='small')
plt.xlim(0,35)
plt.ylim(0,200000)
plt.tight_layout()
plt.show()

"""
apexbi = df["Apex w/bipod"]
apexbiculm = df["Apex w/bipod"].cumsum()
print(apexbi)
print(apexbiculm)

sns.set(style="whitegrid")
plt.figure(figsize=(8, 5))

sns.lineplot(x=time, y=apexbiculm)

plt.title("Total Damage vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Total Damage")
plt.tight_layout()
plt.show()
"""


"""
apexbidps = apexbiculm / time 

sns.set(style="whitegrid")
plt.figure(figsize=(8, 5))

sns.lineplot(x=time, y=apexbidps)

plt.title("Total Damage vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Total Damage")
plt.tight_layout()
plt.show()
"""
