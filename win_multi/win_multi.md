# Windowsプログラムとメッセージを送受信する

[README](../README.md)

---

## Objectives

WindowsからTCP/IPでROSトピックをパブリッシュする。また、ROSからパブリッシュしたトピックをWindowsで受信する。
ただし、Windows側ではプログラムを２つあらかじめ起動しておく。それぞれのWindowsプログラムとの通信に使うトピック名を個別に用意しておことでROS側からどのWindowsプログラムと通信するかを指定できる。

## Prerequisite

You have to finish all of [robots](https://github.com/oit-ipbl/robots) and [image processing](https://github.com/oit-ipbl/image_processing).
