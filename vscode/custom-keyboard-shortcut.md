# Cutomize your keyboard shortcuts

- 키보드 단축키 설정: `Ctrl+K Ctrl+S`
- 우상단 `Open keyboard shortcut(JSON)` 버튼 눌러 `keybindings.json` 파일 열기
- 아래와 같은 형식으로 입력: 예시는 alt키를 조합해 vim처럼 커서 이동하는 단축키 생성
    ```json
    // Place your key bindings in this file to override the defaults
    [
        {
            "key": "alt+l",
            "command": "cursorMove",
            "args": {
                "to": "right"
            },
            "when": "editorTextFocus"
        },
        {
            "key": "alt+h",
            "command": "cursorMove",
            "args": {
                "to": "left"
            },
            "when": "editorTextFocus"
        },
        {
            "key": "alt+j",
            "command": "cursorMove",
            "args": {
                "to": "down"
            },
            "when": "editorTextFocus"
        },
        {
            "key": "alt+k",
            "command": "cursorMove",
            "args": {
                "to": "up"
            },
            "when": "editorTextFocus"
        }
    ]
    ```

- 참고
    - https://code.visualstudio.com/docs/getstarted/tips-and-tricks#_customization
    - https://code.visualstudio.com/docs/getstarted/keybindings#_keyboard-shortcuts-reference
    - https://code.visualstudio.com/api/references/commands
    - https://stackoverflow.com/questions/57436122/vs-code-keyboard-shortcut-to-move-cursor-to-the-center-of-current-screen-after