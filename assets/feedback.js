for (const question of document.querySelectorAll('.question')) {
  const output = question.querySelector('output');
  const options = question.querySelectorAll('button[data-feedback]');
  for (const option of options) {
    option.addEventListener('click', () => {
      for (const other of options) {
        other.setAttribute('aria-pressed', String(other === option));
      }
      output.textContent = option.dataset.feedback;
    });
  }
}
