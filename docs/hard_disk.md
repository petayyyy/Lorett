# Raspberry conect usb hard disk
```bash
sudo fdisk -l
```
![image](https://user-images.githubusercontent.com/47917455/140748436-72aad4d7-9507-4155-b95e-1b2329c501bc.png)

```bash
cd //
sudo mkdir mnt/usb
sudo chown pi:pi /mnt/usb
sudo blkid
```
![image](https://user-images.githubusercontent.com/47917455/140748498-a55e1e12-2c97-4526-ae21-0abf7fdd27f1.png)

```bash
sudo nano /etc/fstab
```

![image](https://user-images.githubusercontent.com/47917455/140748547-da157e8e-1bcf-41b4-9a44-734ad9ef71c0.png)

```bash
UUID=5C81-CFF0 /mnt/usb vfat auto,nofail,noatime,users,rw,uid=pi,gid=pi 0 0
```
### Проверка читаеться ли флешка
```bash
lsblk
```
### Если не читается, то ввелите команду и произведите проверку снова
```bash
sudo mount -a
```
### Если все этапы выполнены то теперь в папке /mnt/usb храняться данные флешки

### Ссылка на доп функцию
https://qastack.ru/ubuntu/17275/how-to-show-the-transfer-progress-and-speed-when-copying-files-with-cp
