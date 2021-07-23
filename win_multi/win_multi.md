# Message exchange between a ROS node and multi Windows programs

[README](../README.md)

---

## Objectives

Let's Make multiple image processing programs on windows and a ROS node which can communicate with each other via TCP/IP.

ここでは複数の画像処理プログラムとROSノードを連携させる方法を学修する

## Prerequisite

You have to finish all of [robots](https://github.com/oit-ipbl/robots), [image processing](https://github.com/oit-ipbl/image_processing), and [Message exchange between a Windows program and a ROS node](win_single/win_single.md)

## Practice1(Single image processing program and a ros node)
### Make a Windows side python program
- 以下の2つのファイルを`code`フォルダ(on windows)に保存しましょう． 
  - ファイルをダウンロードしたい場合はリンクをクリックしてから，`Raw`をクリックしてダウンロードしましょう.
  - [start_on_windows_single.py](./win/start_on_windows_single.py)
    - Windowsで最初に起動し，ROSからのメッセージを待つ．すべての画像処理プログラムはこのモジュールから呼び出される
  - [show_hand_game_win.py](./win/show_hand_game_win.py)
    - ROSとコミュニケーションする画像処理プログラム

#### show_hand_game_win.py
- このプログラムはmediapipesを利用して，ROSが指示した手をユーザがカメラに翳すゲームです
- もしあなたがこのプログラムをテストしたければ，あなたはこのプログラムを下記コマンドで実行できる．
  - このコマンドを実行すると`demo()`が呼ばれ，ROSと通信せず`show_hand_game_win.py`の動作をテストできる
  - あなたが画像処理プログラムを作成する場合も，単独で実行できる関数(e.x. demo())を用意することを強くお勧めする．

```sh
python show_hand_game_win.py
```

- このプログラムの start_game functionは `start_on_windows_single.py` から呼び出される．
  - start_game functionでは，ROSとの間でメッセージの送受信が行われる．例えば，`message_from_ros = ros_bridge_tcp.wait_response(pub_msg, hand_types, timeout=30)` では，pub_msgをROSに送信し，hand_typesに含まれる文字列がROSから返ってくることを最大30秒待ち，返り値をmessage_from_rosに保存する．
    - 第二引数(hand_types)は文字列のリストで，wait_responseはその中に格納されている文字列と完全一致する場合のみ受け取ります．
  - ここで，ROSと交換するメッセージにはこのプログラムに属するメッセージであることを示すprefix(e.x. [shg])を付与することを我々は強く推奨する．
```python
def start_game(topic_name_from_win, ros_bridge_tcp):
    # definition of message types receiving from ros
    hand_types = ["[shg]left", "[shg]right"]

    # Send start message and wait hand type selected by ROS
    pub_msg = {
        "op": "publish",
        "topic": topic_name_from_win,
        "msg": {"data": "[shg]OK_start"}
    }
    message_from_ros = ros_bridge_tcp.wait_response(pub_msg, hand_types, timeout=30)
    if message_from_ros:
        print("\nReceive from ROS:", message_from_ros, "\n")

    # Decide your hand
    window_message = "show only your " + message_from_ros + " hand!"
    left, right = show_hand(window_message)

    # Judge
    result = judge_game(left, right, message_from_ros)
    pub_msg = {
        "op": "publish",
        "topic": topic_name_from_win,
        "msg": {"data": result}
    }
    # Send game result to ROS.
    print(result)
    # In this program, ros never return to the following wait_response
    ros_bridge_tcp.wait_response(pub_msg, ["[shg]OK_result"], timeout=5)
```

#### start_on_windows_single.py
- `show_hand_game_win.py`が正常に実行できるかを確認できたら，`start_on_windows_single.py`に`show_hand_game_win.py`を呼び出す処理を追加し，下記コマンドを実行してROSとの通信を確認すると良い
  - 現在の仕様では，`start_on_windows_single.py`は`while True:`でROSからのメッセージ送信を無限ループで待機するようになっており，ROSから"The end"というメッセージが届くと無限ループを終了する．

```sh
python start_on_windows_single.py
```

- 新しくROSとコミュニケーションする画像処理プログラム(e.x. `show_hand_game_win.py`)を追加する場合は`start_on_windows_single.py`の下記部分を編集しなければならない
  - `if message['msg']['data'] == "[shg]start show hand game":` はROSから`[shg]start show hand game`というメッセージが届いたときに実行される条件式である
  - `shg.start_game(topic_name_from_win, ros_bridge_tcp)` はROSとコミュニケーションする画像処理プログラム（すなわち`show_hand_game_win.py`のstart_game functionを呼ぶ．
    - `import show_hand_game_win as shg` is also required.


```python
    while True:
        messages = ros_bridge_tcp.wait() #Wait for message from ROS and assign the response into 'messages'
        for message in messages:
            # If message['msg']['data'] is "[shg]start show hand game", then shg.star_game is invoked
            if message['msg']['data'] == "[shg]start show hand game":
                shg.start_game(topic_name_from_win, ros_bridge_tcp)
            if message['msg']['data'] == "The end":
                pub_msg = {
                    "op": "publish",
                    "topic": topic_name_from_win,
                    "msg": {"data": "Every game has finished!"}
                }
                print("Every game has finished!")
                # Send message "Every game has finished!" to ros.
                ros_bridge_tcp.wait_response(pub_msg, ["Every game has finished"], timeout=5)
                break
        else:
            continue
        break
```

### Make a ROS node
- Open `~/catkin_ws/` by Visual Studio Code editor, and add the following files into `~/catkin_ws/src/oit_pbl_ros_samples/scripts/`. See [Developing inside the ROS container with VSCode](https://github.com/oit-ipbl/portal/blob/main/setup/remote_with_vscode.md).

- 以下の2つのファイルを`~/catkin_ws/src/oit_pbl_ros_samples/scripts/`フォルダ(on ROS)に保存しましょう． 
  - ファイルをダウンロードしたい場合はリンクをクリックしてから，`Raw`をクリックしてダウンロードしましょう.
  - [start_on_ros_single.py](./ros/start_on_ros_single.py)
    - ROSコンテナ内で起動され，Windows側pythonプログラムに対してメッセージを投げる．以下の`show_hand_game_ros.py`を呼び出す．ROS側プログラム．
  - [show_hand_game_ros.py](./ros/show_hand_game_ros.py)
    - `show_hand_game_win.py`とコミュニケーションするROS側プログラム

#### show_hand_game_ros.py
- このプログラムの `play_show_hand_game` functionが`show_hand_game_win.py`と通信を行う．
- 通常，このプログラムは`start_on_ros_single.py` からモジュールとして呼び出されるが，デモ用に単独で呼び出すこともできる．`show_hand_game_ros.py`の動作のみを確認したい場合には，以降で `rosrun`コマンドを実行する際に，`show_hand_game_ros.py`を直接指定すると良い．
- このファイルを単独で実行するためには下記のStepが必要
```sh
chmod u+x show_hand_game_ros.py
roslaunch oit_stage_ros navitation.launch
```
- 次にWindows側のプログラムをWindowsで起動する．
```sh
python start_on_windows_single.py
```

- 次に以下のコマンドをROSコンテナ内で実行するとROSを通して`demo()` functionが呼び出され，ROSとWindowsの間で通信が行われる．
  - `show_hand_game_ros.py`のみを実行した場合，終了処理がWindows側に送信されないため，`start_on_windows_single.py`が終了しない．

```sh
rosrun oit_pbl_ros_samples show_hand_game_ros.py
``` 

- `show_hand_game_ros.py`におけるROSとWindowsとの通信処理は以下のとおり．コメントをよく読んで理解すること．

```python
def play_show_hand_game():
    rospy.sleep(3) 
    node_name = rospy.get_name()
    # Prepare to play show hand game
    # Specify topic names to commnicate with show hand game
    to_win_pub = rospy.Publisher("/from_ros", String, queue_size=1)
    messenger = RosWinMessenger(to_win_pub, "/from_windows")
    # Start game sequence
    rospy.loginfo("%s:Try to start show hand game", node_name)

    # Send game start signal to Windows, and wait Windows side response.
    message_from_win = messenger.wait_response(
        "[shg]start show hand game", ["[shg]OK_start"], timeout=30)
    if message_from_win:
        rospy.loginfo("%s:Receive from win:%s",
                          node_name, message_from_win)
    else:
        rospy.logerr("%s:Timeout. can't start show hand game on windows", node_name)
        return "[shg]timeout"

    # Select robot's hand_type
    hand_types = ["[shg]left", "[shg]right"]
    hand_type = random.choice(hand_types)
    rospy.loginfo("%s:Robot selects '%s'", node_name, hand_type)

    # Send hand_type chosen by robot to windows show hand game, and wait game result
    message_from_win = messenger.wait_response(
        hand_type, ["[shg]correct", "[shg]wrong"], timeout=30)
    if message_from_win:
        rospy.loginfo("%s:Receive from win:%s",
                    node_name, message_from_win)
    else:
        rospy.logerr(
            "%s:Timeout. can't get show hand game result on windows", node_name)
        return "[shg]timeout"
    rospy.sleep(3)
    return message_from_win       
```

#### start_on_ros_single.py
- `show_hand_game_ros.py`が正常に実行できるかを確認できたら，`start_on_ros_single.py`に`show_hand_game_ros.py`を呼び出す処理を追加し，下記コマンドを実行してWindows側プログラム(`show_hand_game_win.py`)との通信を確認すると良い
  - もし，`roslaunch oit_stage_ros navitation.launch`を実行していない場合は実行し，別のターミナルでコマンドを実行する

```sh
chmod u+x show_hand_game_ros.py
rosrun oit_pbl_ros_samples start_on_ros_single.py
```

- 新しくWindows側（`show_hand_game_win.py`）とコミュニケーションするROSプログラム(e.x. `show_hand_game_ros.py`)を追加する場合は`start_on_ros_single.py`の下記部分を編集しなければならない
  - `result = shgr.play_show_hand_game()` では`show_hand_game_ros.py`をモジュールとして呼び出している
    - `import show_hand_game_ros as shgr` is also required.


```python
def process():
    rospy.sleep(10)
    node_name = rospy.get_name()

    print("---shg---")
    result = shgr.play_show_hand_game()
    rospy.sleep(5)
    end_game()
    rospy.loginfo("/* GAME:%s */", result)
```

## Exercise (add another service)
### Windows side
- 先ほどのpracticeではshow_hand_gameとROSのコミュニケーションを実装しました．このExerciseでは，show_hand_gameに以下のbright_darkゲームを追加してください．
- まず，`bright_dark_game_win.py`をダウンロードし，`code`フォルダ(on windows)に保存しましょう． 
  - ファイルをダウンロードしたい場合はリンクをクリックしてから，`Raw`をクリックしてダウンロードしましょう.
  - [bright_dark_game_win.py](./win/bright_dark_game_win.py)
    - ROSとコミュニケーションする画像処理プログラム
- このプログラムは，ROSが指示する"bright"と"dark"に合わせて，カメラの映像を明るくしたり暗くしたりするゲームです．カメラを照明に向けたり，カメラを手で覆ったりして明るさを調整してみましょう．思い通りにbright/darkの切り替えができない場合は`judge_game`関数内のパラメータを変更してみましょう．

- 保存したら`bright_dark_game_win.py`を実行し，正常に動作するか確認しましょう．
  - たまに正常に動作しない場合があります．その場合はWindows Terminalを再起動してみよう．

```sh
python bright_dark_game_win.py
```
- 上記コマンドが正常に実行できることを確認した後，`start_on_windows_single.py`をコピーし，`start_on_windows_multi.py`に名前を変えて`code`フォルダ(on Windows)に保存しましょう．
- 次に，`start_on_windows_multi.py`の`shg.start_game(topic_name_from_win, ros_bridge_tcp)`の次の行に下記のif文を追加し，`import bright_dark_game_win as bdg`のimport文をプログラムの冒頭に追加しましょう．
  - 以下のコードはbright_dark_game_win.pyの`start_game` functionを呼び出す処理です．

```python
            if message['msg']['data'] == "[bdg]start bright dark game":
                bdg.start_game(topic_name_from_win, ros_bridge_tcp)
```

### ROS side

- 次に`bright_dark_game_ros.py`をダウンロードし，`~/catkin_ws/src/oit_pbl_ros_samples/scripts/`フォルダ(on ROS)に保存しましょう． 
  - [bright_dark_game_ros.py](./ros/bright_dark_game_ros.py)
    - `bright_dark_game_win.py`とコミュニケーションするROS側プログラム
    - Open `~/catkin_ws/` by Visual Studio Code editor, and add the following files into `~/catkin_ws/src/oit_pbl_ros_samples/scripts/`. See [Developing inside the ROS container with VSCode](https://github.com/oit-ipbl/portal/blob/main/setup/remote_with_vscode.md).
    - ファイルをダウンロードしたい場合はリンクをクリックしてから，`Raw`をクリックしてダウンロードしましょう.
- `bright_dark_game_ros.py`をROSに正しく配置できたら，下記コマンドを実行して，正常に動作するか確認しましょう
- 以下のコマンドをROSコンテナ内で実行する
```sh
chmod u+x bright_dark_game_ros.py
roslaunch oit_stage_ros navitation.launch
```
- 次にWindows側のプログラムをWindowsで起動する．
```sh
python start_on_windows_multi.py
```

- 次に以下のコマンドをROSコンテナ内で実行するとROSを通して`demo()` functionが呼び出され，ROSとWindowsの間で通信が行われる．
  - `bright_dark_game_ros.py`のみを実行した場合，終了処理がWindows側に送信されないため，`start_on_windows_multi.py`が終了しない．`start_on_windows_multi.py`を終了したければ，Ctr＋Cを送ること．

```sh
rosrun oit_pbl_ros_samples bright_dark_game_ros.py 
``` 

- `bright_dark_game_ros.py`が正常に実行できるかを確認できたら，`start_on_ros_single.py`をコピーした`start_on_ros_multi.py`を`~/catkin_ws/src/oit_pbl_ros_samples/scripts/`フォルダ(on ROS)に保存しましょう． 
- `start_on_ros_multi.py`に`bright_dark_game_ros.py`を呼び出す処理を追加する
  - `start_on_ros_multi.py`の`process()`内の`end_game()`の前に下記snippetを追加しよう．これは`show_hand_game_ros.py`の`play_show_hand_game()`の次に`bright_dark_game_ros.py`の`play_bright_dark_game()`を呼び出し，結果を受け取るという処理を示している．
  - `import bright_dark_game_ros as bdg` is also required.

```python
    print("---bdg---")
    result_bdg = bdg.play_bright_dark_game()
    rospy.sleep(5)
```

下記コマンドを実行して複数のWindows側プログラムとの通信がうまくいくか確認すること
  - もし，`roslaunch oit_stage_ros navitation.launch`を実行していない場合は実行し，別のターミナルを開いてコマンドを実行すること

- まずWindows側のプログラムをWindowsで起動する．
```sh
python start_on_windows_multi.py
```

- 次に`start_on_ros_multi.py`をROSコンテナ内で実行する
```sh
rosrun oit_pbl_ros_samples start_on_ros_multi.py
```
- show hand gameとbright dark gameがROSとWindows側プログラム間で通信しながら順番に実施されることを確認すること


## Exercise (game and navigation))

Add navigation function into Exercise(add another service) programs. See [Robot control 3](https://github.com/oit-ipbl/robots/blob/main/robot_control/robot_control_03.md#robot-control-3).

