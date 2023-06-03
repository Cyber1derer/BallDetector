{
 "cells": [
  {
   "cell_type": "code",
   "execution_count": 1,
   "metadata": {},
   "outputs": [],
   "source": [
    "def DeleteTrash(pathTxtFile):\n",
    "    with open (pathTxtFile, 'r') as f:\n",
    "        old_data = f.read()\n",
    "        new_data = old_data.replace('[', '')\n",
    "        new_data = new_data.replace(']', '')\n",
    "        new_data = new_data.replace(',', '')\n",
    "        new_data = new_data.replace(';', '')\n",
    "        new_data = new_data.replace('(', '')\n",
    "        new_data = new_data.replace(')', '')\n",
    "    with open (pathTxtFile, 'w') as f:\n",
    "        f.write(new_data)"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 2,
   "metadata": {},
   "outputs": [],
   "source": [
    "import numpy as np\n",
    "import matplotlib.pyplot as plt\n",
    "from ipywidgets import interact\n",
    "\n",
    "FindCord = np.loadtxt(r\"./BallDetectorLogMomentWithCameraMatrix.txt\")\n",
    "BlenderCord63 = np.loadtxt(r\"./LogBlenderCoord63video.txt\", delimiter=',')\n",
    "\n"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 3,
   "metadata": {},
   "outputs": [],
   "source": [
    "def paint(TrueCord = FindCord, FindCord = BlenderCord63):\n",
    "    plt.plot(FindCord, label = \"Find\")\n",
    "    plt.plot(TrueCord, label = \"True\")\n",
    "    plt.grid()\n",
    "    plt.legend()\n",
    "    plt.show()\n"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 4,
   "metadata": {},
   "outputs": [
    {
     "data": {
      "image/png": "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Ef+gIiIiIiKWURERERERy6iIiIiIiGVURERERMQyKiIiIiJiGRURERERsYyKiIiIiFhGRUREREQsoyIiIiIillEREREREcuoiIiIiIhlVERERETEMuFWByiOaZoAZGVllfq2XS4Xubm5ZGVlYbfbS337gU7z8U4z8k4z8k4zKp7m450/zuj83+3zf8eL49dFJDs7G4D4+HiLk4iIiEhJZWdnU758+WLXMczLqSsW8Xg8HD58mNjYWAzDKNVtZ2VlER8fz4EDB4iLiyvVbQcDzcc7zcg7zcg7zah4mo93/jgj0zTJzs6mVq1a2GzFnwXi16+I2Gw26tSp49N9xMXF+c0vzh9pPt5pRt5pRt5pRsXTfLzztxl5eyXkPJ2sKiIiIpZRERERERHLhGwRcTgcPP/88zgcDquj+CXNxzvNyDvNyDvNqHiaj3eBPiO/PllVREREglvIviIiIiIi1lMREREREcuoiIiIiIhlVERERETEMiFZRMaNG0f9+vWJjIwkOTmZlStXWh3JMkuWLOGmm26iVq1aGIbBZ599dsH1pmny3HPPUbNmTaKioujZsyc7duywJqwFxowZQ/v27YmNjaVatWrccsstbNu27YJ18vPzGT58OJUrVyYmJoY//vGPHDt2zKLEZW/8+PG0bt266MOUUlJSmDdvXtH1oT6fS3nllVcwDINHHnmkaFmoz2nUqFEYhnHBT9OmTYuuD/X5ABw6dIj+/ftTuXJloqKiaNWqFatXry66PlCfr0OuiHz00UeMHDmS559/nrVr19KmTRv69OlDRkaG1dEscfbsWdq0acO4ceMuef1rr73GO++8w4QJE1ixYgXlypWjT58+5Ofnl3FSayxevJjhw4ezfPlyFixYgMvlonfv3pw9e7ZonUcffZT//Oc/fPzxxyxevJjDhw9z2223WZi6bNWpU4dXXnmFNWvWsHr1aq6++mpuvvlmNm3aBGg+v7Rq1Sref/99WrdufcFyzQlatGjBkSNHin6WLl1adF2oz+f06dN07twZu93OvHnz2Lx5M2+88QYVK1YsWidgn6/NENOhQwdz+PDhRZfdbrdZq1Ytc8yYMRam8g+AOXfu3KLLHo/HrFGjhvn6668XLTtz5ozpcDjMf/7znxYktF5GRoYJmIsXLzZN89w87Ha7+fHHHxets2XLFhMwly1bZlVMy1WsWNGcNGmS5vML2dnZZuPGjc0FCxaY3bt3N0eMGGGaph5Hpmmazz//vNmmTZtLXqf5mOaTTz5pdunS5VevD+Tn65B6RaSgoIA1a9bQs2fPomU2m42ePXuybNkyC5P5pz179nD06NEL5lW+fHmSk5NDdl6ZmZkAVKpUCYA1a9bgcrkumFHTpk2pW7duSM7I7XYzZ84czp49S0pKiubzC8OHD+eGG264YB6gx9F5O3bsoFatWjRs2JB+/fqxf/9+QPMB+OKLL0hKSuL222+nWrVqJCQkMHHixKLrA/n5OqSKyIkTJ3C73VSvXv2C5dWrV+fo0aMWpfJf52eieZ3j8Xh45JFH6Ny5My1btgTOzSgiIoIKFSpcsG6ozSg9PZ2YmBgcDgf33Xcfc+fOpXnz5prPz8yZM4e1a9cyZsyYi67TnCA5OZlp06bxzTffMH78ePbs2UPXrl3Jzs7WfIDdu3czfvx4GjduzLfffsv999/Pww8/zPTp04HAfr7262/fFfEnw4cPZ+PGjRcct5ZzmjRpwrp168jMzOSTTz4hNTWVxYsXWx3Lbxw4cIARI0awYMECIiMjrY7jl6677rqif7du3Zrk5GTq1avHv/71L6KioixM5h88Hg9JSUm8/PLLACQkJLBx40YmTJhAamqqxel+n5B6RaRKlSqEhYVddKb1sWPHqFGjhkWp/Nf5mWhe8OCDD/Lll1/y/fffU6dOnaLlNWrUoKCggDNnzlywfqjNKCIigkaNGpGYmMiYMWNo06YNf//73zWf/1mzZg0ZGRm0a9eO8PBwwsPDWbx4Me+88w7h4eFUr15dc/qFChUqcOWVV7Jz5049joCaNWvSvHnzC5Y1a9as6PBVID9fh1QRiYiIIDExkYULFxYt83g8LFy4kJSUFAuT+acGDRpQo0aNC+aVlZXFihUrQmZepmny4IMPMnfuXL777jsaNGhwwfWJiYnY7fYLZrRt2zb2798fMjO6FI/Hg9Pp1Hz+55prriE9PZ1169YV/SQlJdGvX7+if2tOF8rJyWHXrl3UrFlTjyOgc+fOF310wPbt26lXrx4Q4M/XVp8tW9bmzJljOhwOc9q0aebmzZvNe+65x6xQoYJ59OhRq6NZIjs720xLSzPT0tJMwHzzzTfNtLQ0c9++faZpmuYrr7xiVqhQwfz888/NDRs2mDfffLPZoEEDMy8vz+LkZeP+++83y5cvby5atMg8cuRI0U9ubm7ROvfdd59Zt25d87vvvjNXr15tpqSkmCkpKRamLltPPfWUuXjxYnPPnj3mhg0bzKeeeso0DMOcP3++aZqaz6/5+btmTFNzeuyxx8xFixaZe/bsMX/88UezZ8+eZpUqVcyMjAzTNDWflStXmuHh4ebf/vY3c8eOHeaHH35oRkdHm7NmzSpaJ1Cfr0OuiJimab777rtm3bp1zYiICLNDhw7m8uXLrY5kme+//94ELvpJTU01TfPcW8KeffZZs3r16qbD4TCvueYac9u2bdaGLkOXmg1gTp06tWidvLw884EHHjArVqxoRkdHm7feeqt55MgR60KXscGDB5v16tUzIyIizKpVq5rXXHNNUQkxTc3n1/yyiIT6nO68806zZs2aZkREhFm7dm3zzjvvNHfu3Fl0fajPxzRN8z//+Y/ZsmVL0+FwmE2bNjU/+OCDC64P1OdrwzRN05rXYkRERCTUhdQ5IiIiIuJfVERERETEMioiIiIiYhkVEREREbGMioiIiIhYRkVERERELKMiIiIiIpZRERERERHLqIiIiIiIZVRERERExDIqIiIiImIZFRERERGxzP8DkC0OKlvkNrUAAAAASUVORK5CYII=",
      "text/plain": [
       "<Figure size 640x480 with 1 Axes>"
      ]
     },
     "metadata": {},
     "output_type": "display_data"
    }
   ],
   "source": [
    "w = interact(paint(BlenderCord63[:,0], FindCord[:,0]) )"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 5,
   "metadata": {},
   "outputs": [
    {
     "data": {
      "text/plain": [
       "<ipywidgets.widgets.interaction._InteractFactory at 0x2611810b340>"
      ]
     },
     "metadata": {},
     "output_type": "display_data"
    }
   ],
   "source": []
  },
  {
   "cell_type": "code",
   "execution_count": null,
   "metadata": {},
   "outputs": [],
   "source": []
  }
 ],
 "metadata": {
  "kernelspec": {
   "display_name": "Python 3 (ipykernel)",
   "language": "python",
   "name": "python3"
  },
  "language_info": {
   "codemirror_mode": {
    "name": "ipython",
    "version": 3
   },
   "file_extension": ".py",
   "mimetype": "text/x-python",
   "name": "python",
   "nbconvert_exporter": "python",
   "pygments_lexer": "ipython3",
   "version": "3.10.8"
  }
 },
 "nbformat": 4,
 "nbformat_minor": 4
}