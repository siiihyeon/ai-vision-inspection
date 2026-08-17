# Git 브랜치·커밋·병합 빠른 안내

기준 저장소: `~/ai-vision-inspection`

## 1. 저장소로 이동

```bash
cd ~/ai-vision-inspection
```

다음 Git 명령은 WSL 저장소 최상위 폴더에서 수행합니다.

## 2. 작업 시작 전 main 최신화

```bash
git fetch origin
git status --short --branch
git switch main
git pull origin main
```

GitHub의 최신 `main`을 WSL 저장소에 내려받습니다.

## 3. 새 작업 브랜치 생성

```bash
git switch -c feature/작업이름
```

최신 main을 기준으로 새 브랜치를 만들고 바로 이동합니다.

예시:

```bash
git switch -c feature/master-product-flow
```

브랜치 이름은 `feature/`, `fix/`, `docs/`, `chore/` 등 작업 성격에 맞게 정합니다.

## 4. main에서 이미 파일을 수정한 경우

```bash
git switch -c feature/작업이름
```

아직 커밋하지 않은 변경은 일반적으로 새 브랜치로 함께 이동합니다.

## 5. 변경 파일 확인

```bash
git status
git diff
```

커밋 전에 변경된 파일과 실제 수정 내용을 확인합니다.

## 6. 커밋할 파일 선택

```bash
git add 경로/파일명
```

검토한 파일만 staging 영역에 추가합니다.

여러 파일 예시:

```bash
git add ros2_ws/tools/build_helper.md ros2_ws/tools/git_helper.md
```

## 7. 커밋 직전 내용 확인

```bash
git diff --staged
```

실제로 커밋에 들어갈 변경만 마지막으로 확인합니다.

## 8. 커밋 생성

```bash
git commit -m "docs: add build and git workflow helpers"
```

현재 브랜치에 작업 시점과 변경 목적을 기록합니다.

자주 쓰는 접두어:

```text
feat: 새 기능
fix: 오류 수정
docs: 문서 수정
test: 테스트 추가·수정
refactor: 동작 변경 없는 구조 개선
chore: 설정·정리 작업
```

## 9. GitHub에 브랜치 push

```bash
git push -u origin feature/작업이름
```

처음 한 번 `-u`로 연결하면 이후에는 현재 브랜치에서 `git push`만 입력해도 됩니다.

## 10. Pull Request 생성·병합

```text
GitHub 저장소 접속
→ Compare & pull request
→ base: main / compare: 작업 브랜치 확인
→ Create pull request
→ 변경 파일과 자동 검사 확인
→ Merge pull request
→ Confirm merge
```

작업 브랜치를 바로 main에 덮어쓰지 않고 GitHub에서 변경 내용을 확인한 뒤 병합합니다.

## 11. 병합 결과를 WSL에 내려받기

```bash
git switch main
git pull origin main
```

GitHub에서 병합된 최신 main을 로컬 WSL 저장소에 반영합니다.

## 12. 사용이 끝난 로컬 브랜치 삭제

```bash
git branch -d feature/작업이름
```

main에 정상 병합된 작업 브랜치를 로컬에서 정리합니다.

## 팀원이 작업 중 main을 갱신한 경우

```bash
git switch main
git pull origin main
git switch feature/작업이름
git merge main
```

최신 main을 현재 작업 브랜치에 합치고 충돌이 있으면 해당 파일만 해결합니다.

충돌 해결 후:

```bash
git add 충돌을_해결한_파일
git commit
git push
```

해결한 내용을 커밋하고 작업 브랜치를 다시 GitHub에 올립니다.

## 작업 중 기억할 규칙

- 작업 시작 전 `main`에서 pull한 뒤 새 브랜치를 만듭니다.
- 커밋 전에 `git status`, `git diff`, `git diff --staged`를 확인합니다.
- 가능하면 `git add .`보다 수정한 파일 경로를 직접 지정합니다.
- 팀원이 함께 쓰는 `main`에 직접 push하지 않습니다.
- `git reset --hard`와 강제 push는 사용하지 않습니다.
- GitHub에서 병합한 뒤 WSL의 `main`도 반드시 pull합니다.

