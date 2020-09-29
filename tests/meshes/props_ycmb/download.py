# -*- coding: utf-8 -*-
"""
Created on Thu Sep 24 18:05:42 2020

@author: Ramona
"""

import os
import requests
from time import time
from multiprocessing.pool import ThreadPool
import wget
import urllib.request, urllib.parse, urllib.error
from bs4 import BeautifulSoup
import ssl

def url_response(url):
    path, url = url
    r = requests.get(url, stream = True)
    with open(path, 'wb') as f:
        for ch in r:
            f.write(ch)


def url_response2(url):
    wget.download(url[1])

# Ignore SSL certificate errors
ctx = ssl.create_default_context()
ctx.check_hostname = False
ctx.verify_mode = ssl.CERT_NONE

url = 'http://ycb-benchmarks.s3-website-us-east-1.amazonaws.com/'
html = urllib.request.urlopen(url, context=ctx).read()
soup = BeautifulSoup(html, 'html.parser')

# Retrieve all of the anchor tags
tags = soup('a')

anchors = [tag.get('href', None) for tag in tags]

keywords = {'processed':'meshes', 'rgb':['highres','rgbd'], 'model':['16k','64k','512k']}

links_processed = [it for it in anchors if it.find(keywords['processed'])>0]
links_rgb_hr = [it for it in anchors if it.find(keywords['rgb'][0])>0]
links_rgb_d = [it for it in anchors if it.find(keywords['rgb'][1])>0]
links_models_16k = [it for it in anchors if it.find(keywords['model'][0])>0]
links_models_64k = [it for it in anchors if it.find(keywords['model'][1])>0]
links_models_512k = [it for it in anchors if it.find(keywords['model'][2])>0]


urls = links_models_16k # Change for whatever set you want

urls = [['Event'+str(i),url+urls[i]] for i in range(len(urls))]

print('The following models will be downloaded:')

[print(mod) for mod in urls]

start = time()

print (len(urls))
for i in len(urls)
    wget.download(urls[i][1])
#url_response2(string(urls[0][1]))
#wget.download(urls[0][1])
#ThreadPool(len(urls)).imap_unordered(url_response2, urls)

print(f"Time to download: {time() - start}")
