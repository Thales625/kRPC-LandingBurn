from datetime import datetime

class DataLogger:
    def __init__(self) -> None:
        self.format = 'ut|alt|dist|v_vert|v_hor|delta_speed|pitch'
        self.now = datetime.now().strftime("%d-%m-%Y %H-%M")
        self.filename = f"report/{self.now}.txt"

        self.file = open(self.filename, "w")
        self.file.write(f"{self.format}\n")

        self.ut0 = 0

    def log(self, ut, dist, alt, v_vert, v_hor, delta_speed, pitch):
        self.file.write(f"{(ut-self.ut0):.2f}|{dist:.2f}|{alt:.2f}|{v_vert:.2f}|{v_hor:.2f}|{delta_speed:.2f}|{pitch:.2f}\n")

    def close(self):
        self.file.close()

if __name__ == "__main__":
    DataLogger()