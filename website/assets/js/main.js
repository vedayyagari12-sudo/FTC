(() => {
  const reduceMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;

  // Header state + mobile menu
  const header = document.querySelector('.site-header');
  const menuBtn = document.querySelector('.menu-btn');
  const onScroll = () => header.classList.toggle('scrolled', window.scrollY > 8);
  onScroll();
  window.addEventListener('scroll', onScroll, { passive: true });

  const setMenu = (open) => {
    document.body.classList.toggle('menu-open', open);
    menuBtn.setAttribute('aria-expanded', String(open));
    menuBtn.setAttribute('aria-label', open ? 'Close menu' : 'Open menu');
  };
  menuBtn.addEventListener('click', () => setMenu(!document.body.classList.contains('menu-open')));
  document.querySelectorAll('.nav a').forEach((a) => a.addEventListener('click', () => setMenu(false)));
  document.addEventListener('keydown', (e) => { if (e.key === 'Escape') setMenu(false); });

  // Active nav link
  const navLinks = [...document.querySelectorAll('.nav > a[href^="#"]')];
  const sections = navLinks.map((a) => document.querySelector(a.getAttribute('href'))).filter(Boolean);
  const spy = new IntersectionObserver((entries) => {
    entries.forEach((entry) => {
      if (!entry.isIntersecting) return;
      navLinks.forEach((a) => a.classList.toggle('active', a.getAttribute('href') === '#' + entry.target.id));
    });
  }, { rootMargin: '-45% 0px -50% 0px' });
  sections.forEach((s) => spy.observe(s));

  // Reveal on scroll
  const reveals = document.querySelectorAll('.reveal');
  if (reduceMotion || !('IntersectionObserver' in window)) {
    reveals.forEach((el) => el.classList.add('in'));
  } else {
    const io = new IntersectionObserver((entries) => {
      entries.forEach((entry) => {
        if (entry.isIntersecting) { entry.target.classList.add('in'); io.unobserve(entry.target); }
      });
    }, { rootMargin: '0px 0px -8% 0px', threshold: 0.08 });
    reveals.forEach((el) => io.observe(el));
  }

  // Count-up stats
  const counters = document.querySelectorAll('[data-count]');
  if (!reduceMotion) {
    const cio = new IntersectionObserver((entries) => {
      entries.forEach((entry) => {
        if (!entry.isIntersecting) return;
        const el = entry.target;
        const end = Number(el.dataset.count);
        const start = performance.now();
        const dur = 1400;
        const step = (now) => {
          const p = Math.min(1, (now - start) / dur);
          el.textContent = Math.round(end * (1 - Math.pow(1 - p, 3)));
          if (p < 1) requestAnimationFrame(step);
        };
        el.textContent = '0';
        requestAnimationFrame(step);
        cio.unobserve(el);
      });
    }, { threshold: 0.6 });
    counters.forEach((el) => cio.observe(el));
  }

  // Robot photo viewer
  const viewerImg = document.getElementById('viewer-img');
  const thumbs = document.querySelectorAll('.viewer-thumbs button');
  thumbs.forEach((btn) => {
    btn.addEventListener('click', () => {
      thumbs.forEach((b) => b.setAttribute('aria-pressed', String(b === btn)));
      viewerImg.classList.add('swap');
      setTimeout(() => {
        viewerImg.src = btn.dataset.src;
        viewerImg.alt = btn.dataset.alt;
        viewerImg.classList.remove('swap');
      }, reduceMotion ? 0 : 180);
    });
  });

  // Lightbox
  const lb = document.querySelector('.lightbox');
  if (lb && typeof lb.showModal === 'function') {
    const lbImg = lb.querySelector('img');
    const lbCap = lb.querySelector('.lb-cap');
    document.querySelectorAll('.gallery button').forEach((btn) => {
      btn.addEventListener('click', () => {
        const img = btn.querySelector('img');
        lbImg.src = img.src;
        lbImg.alt = img.alt;
        lbCap.textContent = img.alt;
        lb.showModal();
      });
    });
    lb.querySelector('.lb-close').addEventListener('click', () => lb.close());
    lb.addEventListener('click', (e) => { if (e.target === lb) lb.close(); });
  }

  // Contact form -> prefilled email
  const form = document.getElementById('contact-form');
  if (form) {
    const note = form.querySelector('.form-note');
    form.addEventListener('submit', (e) => {
      e.preventDefault();
      let ok = true;
      form.querySelectorAll('[required]').forEach((f) => {
        const valid = f.value.trim() && (f.type !== 'email' || /\S+@\S+\.\S+/.test(f.value));
        f.classList.toggle('invalid', !valid);
        if (!valid) ok = false;
      });
      if (!ok) { note.textContent = 'Please fill in your name, a valid email, and a message.'; return; }
      const d = Object.fromEntries(new FormData(form));
      const subject = `${d.topic}: ${d.name}${d.org ? ' (' + d.org + ')' : ''}`;
      const body = `${d.message}\n\n${d.name}\n${d.email}${d.org ? '\n' + d.org : ''}`;
      window.location.href = `mailto:ftc.torquetyrants@gmail.com?subject=${encodeURIComponent(subject)}&body=${encodeURIComponent(body)}`;
      note.textContent = 'Your email app should open with the message ready to send.';
    });
  }

  // If the 3D module can't load (old browser, blocked CDN), show the logo instead.
  const stage = document.getElementById('robot-stage');
  window.addEventListener('load', () => {
    setTimeout(() => {
      if (stage && !stage.classList.contains('ready')) stage.classList.add('no-webgl');
    }, 4000);
  });

  const year = document.getElementById('year');
  if (year) year.textContent = new Date().getFullYear();
})();
