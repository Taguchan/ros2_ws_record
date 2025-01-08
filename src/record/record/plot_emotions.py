import pandas as pd
import matplotlib.pyplot as plt

# CSVファイルを読み込み
df = pd.read_csv("output/emotion_data.csv")

# プロット
plt.figure(figsize=(20, 8))
for column in df.columns:
    if column != "frame":  # フレーム番号はプロットしない
        plt.plot(df["frame"], df[column], label=column)

# プロットの設定
plt.title("Emotion Changes Over Time", fontsize=20)
plt.xlabel("Frame", fontsize=16)
plt.ylabel("Emotion Score", fontsize=16)
plt.legend(fontsize=12)
plt.grid(True)
plt.savefig("output/emotion_plot.png")
plt.show()

print("プロット画像を output/emotion_plot.png に保存しました。")
