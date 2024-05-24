import os,sys
import math
import numpy as np
import librosa
import librosa.display
import torch
import yaml
from typing import Any, Dict, Optional

import matplotlib.pyplot as plt
from urllib3 import Retry
import matplotlib


from easydict import EasyDict

def load_setting(setting):
    with open(setting, 'r') as f:
        cfg = yaml.load(f, Loader=yaml.FullLoader)
    return EasyDict(cfg)

def xyz2uv_sphere(lats, lons, alts):
    #print(lats, lons)
    z = np.sqrt(lats*lats + lons*lons)
    alpha = np.arctan2(lons, lats) * 180 / np.pi
    beta  = np.arctan2(alts, z) * 180 / np.pi
    r = np.sqrt(alts*alts + z*z)
    return alpha, beta, r

def uv2xyz_sphere(alpha, beta, r):
    alpha = alpha * np.pi / 180
    beta  = beta  * np.pi / 180
    alts = np.sin(beta)  * r
    z = np.cos(beta)
    lons = np.sin(alpha) * z * r
    lats = np.cos(alpha) * z * r
    return lats, lons, alts 

def uv2xyz_torch(alpha, beta, r):
    alpha = alpha * np.pi / 180.
    beta  = beta  * np.pi / 180.
    alts  = torch.sin(beta) * r
    z     = torch.cos(beta)
    lons  = torch.sin(alpha) * z * r
    lats  = torch.cos(alpha) * z * r
    return lats, lons, alts


def gauss_1d(sz, sigma, center):
    k = torch.arange(sz).reshape(1, -1)
    gauss = torch.exp(-1.0 / (2 * sigma ** 2) * (k - center.reshape(-1, 1)) ** 2)
    return gauss

def gauss_2d(sz, sigma, center, end_pad=(0, 0), density=False):
    if isinstance(sigma, (float, int)):
        sigma = (sigma, sigma)
    return gauss_1d(sz[0], sigma[0], center[:, 0]).reshape(center.shape[0], 1, -1) * \
           gauss_1d(sz[1], sigma[1], center[:, 1]).reshape(center.shape[0], -1, 1)

def audio2spectrum(x, sr, nc):
    res = []
    #for i in range(nc):
    res.append(librosa.feature.melspectrogram(
        #y=x[:,i],
        y=x,
        sr=sr,
        n_fft=2048,
        hop_length=1024,
        n_mels=80,
        fmax = 1000
    ))
    return np.stack(res, -1)


def show_spectrum(xs, sr, nc=16):
    #print(x[:100,:])
    fig, axs = plt.subplots(4, 4)
    for i in range(nc):
        x = xs[sr*10: sr*11, i].astype(np.float32)
        res = librosa.feature.melspectrogram(
            y=x,
            sr=sr,
            n_fft=1024,
            hop_length=256,
            n_mels=80,
        )
        print(res.shape, x.shape)
        
        S_dB = librosa.power_to_db(res, ref=np.max)
        print(S_dB.shape)
        img = librosa.display.specshow(S_dB, x_axis='time',
                                y_axis='mel', sr=sr,
                                fmax=8000, ax=axs[i//4,i%4])
        #fig.colorbar(img, ax=ax, format='%+2.0f dB')
        axs[i//4,i%4].set(title='Mel-frequency spectrogram')
    plt.show()  


def cal_lat_dis(lat, lat0):
    return (lat - lat0) * 111320

def cal_lon_dis(lon, lon0, lat0):
    return (lon - lon0) * math.cos(lat0) * 111320 


def coords_trans(dlat, dlon, dalt, theta):
    dlat_ = dlat * np.cos(theta) - dlon * np.sin(theta)
    dlon_ = dlat * np.sin(theta) + dlon * np.cos(theta)
    return dlat_, dlon_, dalt

def plot_3d_trajectory(lat, lon, alt, lat0=None, lon0=None, theta=None):
    if lat0 is not None:
        # 3D Plot of flight data
        dlat = cal_lat_dis(lat, lat0)
        dlon = -1 * cal_lon_dis(lon, lon0, lat0)
        dalt = alt

        if theta is not None:
            dlat_ = dlat * np.cos(theta) - dlon * np.sin(theta)
            dlon_ = dlat * np.sin(theta) + dlon * np.cos(theta)
            dlat = dlat_
            dlon = dlon_
    else:
        dlat, dlon, dalt = lat, lon, alt
    fig = plt.figure(figsize=(8,6))
    ax = plt.subplot(111, projection='3d')
    ax.xaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor('white')
    ax.yaxis.pane.fill = False
    ax.yaxis.pane.set_edgecolor('white')
    ax.zaxis.pane.fill = False
    ax.zaxis.pane.set_edgecolor('white')
    ax.grid(False)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.plot(dlat, dlon, dalt)
    plt.show()  



def calibrate_mic_coords(lat1, lon1, lat2, lon2, lat0, lon0):
    alpha = (lon2-lon1)/(lat2-lat1)

    #nlat1 = 



def transform_global2camera(lat1, lon1, lat2, lon2, lat0, lon0):
    dlat1 = cal_lat_dis(lat1, lat0)
    dlon1 = cal_lon_dis(lon1, lon0, lat0)
    dlat2 = cal_lat_dis(lat2, lat0)
    dlon2 = cal_lon_dis(lon2, lon0, lat0)

    theta = np.arctan2(dlon2-dlon1, dlat2-dlat1) * 180 / np.pi

    r = [[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]]

    return r, theta



def save_checkpoint(config, epoch, model, error=0.):
    save_state = {'model': model.state_dict(),
                  'epoch': epoch,
                  'config': config}

    save_path = os.path.join(config.save_path+'/{}'.format(config.config.split('.')[0]), f'ckpt_epoch_{epoch}_{error:.3f}.pth')
    torch.save(save_state, save_path)




def projection_3d2fish(lats, lons, alts, Xf=1920, Yf=1920, aperture=np.pi):
    r = np.sqrt(lats*lats + lons*lons + alts*alts)
    px = lats/r
    py = lons/r 
    pz = -(alts-5)/r 

    r = Xf * np.arctan2(np.sqrt(px*px + pz*pz), py) / aperture
    theta = np.arctan2(pz, px)
    x = r * np.cos(theta) + 960
    y = r * np.sin(theta) + 960

    return np.array(x).astype(np.int), np.array(y).astype(np.int)


def projection_uv2fish(lat, lon, Xf=1920, Yf=1920, aperture=np.pi):
    lat = lat / 180 * np.pi - 0.1
    lon = (lon + 20) / 180 * np.pi - np.pi/2.

    px = np.cos(-lat) * np.sin(lon)
    py = np.cos(-lat) * np.cos(lon)
    pz = np.sin(-lat)

    r = Xf * np.arctan2(np.sqrt(px*px + pz*pz), py) / aperture
    theta = np.arctan2(pz, px)
    x = 1920 - (r * np.cos(theta) + 960)
    y = r * np.sin(theta) + 960

    return np.array(x).astype(np.int), np.array(y).astype(np.int)


def colorize(value, cmap='viridis', vmin=None, vmax=None):
    # normalize
    vmin = value.min() if vmin is None else vmin
    vmax = value.max() if vmax is None else vmax

    if vmin != vmax:
        value = (value - vmin) / (vmax - vmin)  # vmin..vmax
    else:
        # Avoid 0-division
        value = value * 0.

    cmapper = matplotlib.cm.get_cmap(cmap)
    value = cmapper(value, bytes=True)  # ((1)xhxwx4)
    value = value[:, :, :3] # bgr -> rgb
    rgb_value = value[..., ::-1]

    return rgb_value
